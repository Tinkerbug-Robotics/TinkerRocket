import Foundation
import CoreBluetooth

// #526: consumes a file download over an L2CAP CoC channel.
//
// A CBL2CAPChannel exposes an InputStream/OutputStream pair that MUST be serviced
// on a thread with a live run loop. We own a dedicated thread for exactly that, so
// the ~250 s transfer never blocks the main thread and is unaffected by SwiftUI
// work. Bytes -> FileStreamDecoder -> a `.part` file on disk (never an 8.5 MB Data
// in RAM) -> on END, verify byte count + CRC + status, and only then rename to the
// real filename. Anything else is a failure and the caller falls back to GATT.
//
// Backgrounding: a suspended app stops servicing this run loop, so credits dry up
// and the firmware stalls, then aborts after its 30 s budget (bench-confirmed: the
// channel itself survives). We treat L2CAP as a foreground fast path and let the
// caller retry over GATT — so a `noBytes` timer here fails cleanly rather than
// hanging.
// nonisolated: the stream delegate callbacks and file writes run on our own
// background thread with a live run loop, NOT the main actor (the project's
// default). Only the progress/completion hand-offs are dispatched to main.
nonisolated final class L2CAPFileReceiver: NSObject, StreamDelegate {
    enum Failure: Error { case refused, readError, crcMismatch, byteMismatch, streamError, timeout, endStatus }

    private let channel: CBL2CAPChannel
    private let expectedName: String
    private let completion: (Result<URL, Error>) -> Void
    private let progress: (Double) -> Void
    private let sizeHintFallback: Int

    private let decoder = FileStreamDecoder()
    private var thread: Thread?
    private var input: InputStream?
    private var output: OutputStream?
    private var noBytesTimer: DispatchSourceTimer?
    private let noBytesTimeout: TimeInterval = 30.0

    // File + integrity state (touched only on the receiver thread).
    private var partURL: URL?
    private var handle: FileHandle?
    private var runningCrc = Crc32.initial
    private var bytesWritten: UInt32 = 0
    private var finished = false
    private var lastProgressReport = Date.distantPast

    private var readBuf = [UInt8](repeating: 0, count: 8192)

    init(channel: CBL2CAPChannel,
         expectedName: String,
         sizeHint: Int,
         progress: @escaping (Double) -> Void,
         completion: @escaping (Result<URL, Error>) -> Void) {
        self.channel = channel
        self.expectedName = expectedName
        self.sizeHintFallback = sizeHint
        self.progress = progress
        self.completion = completion
        super.init()
        wireDecoder()
    }

    /// Begin consuming. Call once; a receiver is single-shot.
    func start() {
        let t = Thread { [weak self] in
            guard let self = self else { return }
            guard let input = self.channel.inputStream, let output = self.channel.outputStream else {
                self.finish(.failure(Failure.streamError)); return
            }
            self.input = input
            self.output = output
            input.delegate = self
            output.delegate = self
            input.schedule(in: .current, forMode: .default)
            output.schedule(in: .current, forMode: .default)
            input.open()
            output.open()
            self.armNoBytesTimer()
            // Keep the run loop alive until finish() invalidates the sources.
            while !self.finished && RunLoop.current.run(mode: .default, before: .distantFuture) {}
        }
        t.name = "l2cap-file-receiver"
        t.qualityOfService = .userInitiated
        thread = t
        t.start()
    }

    /// Tear down without delivering (used if the app cancels).
    func cancel() { finish(.failure(Failure.streamError)) }

    // MARK: - decoder wiring

    private func wireDecoder() {
        decoder.onBegin = { [weak self] begin in
            guard let self = self else { return }
            if begin.status != 0 {
                // status 2 = INFLIGHT refusal, 3 = read error — not a file.
                self.finish(.failure(begin.status == 2 ? Failure.refused : Failure.readError))
                return
            }
            self.openPartFile()
        }
        decoder.onData = { [weak self] chunk in
            guard let self = self, let handle = self.handle else { return }
            handle.write(chunk)
            self.runningCrc = Crc32.update(self.runningCrc, chunk)
            self.bytesWritten += UInt32(chunk.count)
            self.reportProgress()
        }
        decoder.onEnd = { [weak self] end in
            guard let self = self else { return }
            self.completeFromEnd(end)
        }
        decoder.onError = { [weak self] _ in
            self?.finish(.failure(Failure.streamError))
        }
    }

    // MARK: - StreamDelegate

    func stream(_ aStream: Stream, handle eventCode: Stream.Event) {
        guard aStream === input else { return }   // only the input stream matters
        switch eventCode {
        case .hasBytesAvailable:
            drainInput()
        case .errorOccurred:
            finish(.failure(Failure.streamError))
        case .endEncountered:
            // The stream closed before a valid END record -> failure.
            if !finished { finish(.failure(Failure.streamError)) }
        default:
            break
        }
    }

    private func drainInput() {
        guard let input = input else { return }
        while input.hasBytesAvailable {
            // Use the buffer pointer's OWN count/baseAddress — reading readBuf.count
            // inside its withUnsafeMutableBytes borrow is a Swift exclusivity abort.
            let read = readBuf.withUnsafeMutableBytes { (ptr: UnsafeMutableRawBufferPointer) -> Int in
                guard let base = ptr.baseAddress else { return 0 }
                return input.read(base.assumingMemoryBound(to: UInt8.self), maxLength: ptr.count)
            }
            if read > 0 {
                rearmNoBytesTimer()
                decoder.feed(Data(readBuf[0..<read]))
                if finished { return }
            } else {
                break   // 0 = no more right now; <0 = error, surfaced via .errorOccurred
            }
        }
    }

    // MARK: - file lifecycle

    private func openPartFile() {
        let tmp = FileManager.default.temporaryDirectory
        let url = tmp.appendingPathComponent(expectedName + ".part")
        try? FileManager.default.removeItem(at: url)
        FileManager.default.createFile(atPath: url.path, contents: nil)
        partURL = url
        handle = try? FileHandle(forWritingTo: url)
        if handle == nil { finish(.failure(Failure.readError)) }
    }

    private func completeFromEnd(_ end: FileStreamDecoder.End) {
        if end.status != 0 { finish(.failure(Failure.endStatus)); return }
        if end.bytes != bytesWritten { finish(.failure(Failure.byteMismatch)); return }
        if Crc32.finalize(runningCrc) != end.crc { finish(.failure(Failure.crcMismatch)); return }

        try? handle?.close()
        handle = nil
        guard let part = partURL else { finish(.failure(Failure.readError)); return }
        let final = FileManager.default.temporaryDirectory.appendingPathComponent(expectedName)
        try? FileManager.default.removeItem(at: final)
        do {
            try FileManager.default.moveItem(at: part, to: final)
            finish(.success(final))
        } catch {
            finish(.failure(Failure.readError))
        }
    }

    private func reportProgress() {
        // Throttle to ~10 Hz; size_hint is a hint only (recovered flights under-run).
        let now = Date()
        if now.timeIntervalSince(lastProgressReport) < 0.1 { return }
        lastProgressReport = now
        let total = sizeHintFallback > 0 ? Double(sizeHintFallback) : 0
        let frac = total > 0 ? min(Double(bytesWritten) / total, 1.0) : 0
        DispatchQueue.main.async { [weak self] in self?.progress(frac) }
    }

    // MARK: - timers + teardown

    private func armNoBytesTimer() {
        let t = DispatchSource.makeTimerSource(queue: .global(qos: .userInitiated))
        t.schedule(deadline: .now() + noBytesTimeout)
        t.setEventHandler { [weak self] in self?.finish(.failure(Failure.timeout)) }
        noBytesTimer = t
        t.resume()
    }
    private func rearmNoBytesTimer() {
        noBytesTimer?.schedule(deadline: .now() + noBytesTimeout)
    }

    private func finish(_ result: Result<URL, Error>) {
        if finished { return }
        finished = true
        noBytesTimer?.cancel(); noBytesTimer = nil
        try? handle?.close(); handle = nil
        // On failure, discard the partial file — never leave a truncated download.
        if case .failure = result, let part = partURL { try? FileManager.default.removeItem(at: part) }
        input?.close()
        output?.close()
        DispatchQueue.main.async { [weak self] in self?.completion(result) }
    }
}
