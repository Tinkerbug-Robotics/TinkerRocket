// Transparent USB <-> PX1105R UART bridge for the COCOM bench rig (#491).
//
// Copyright (c) 2026 Tinkerbug Robotics
// MIT License
//
// Why this exists rather than the stock TinkerNav firmware: the TinkerNav
// rover/base sketches consume the GNSS stream into TR_SkyTraqNMEA's parser and
// surface a decoded subset over a web UI --
//
//     while (Serial1.available()) { char c = Serial1.read(); sky.feedChar(c); }
//
// -- so raw sentences never reach USB.  #491 needs the opposite: every byte the
// receiver emits, timestamped on the host, including the GSV/GSA traffic that
// separates a COCOM block (satellites still tracked, position withheld) from a
// loss of lock (C/N0 collapses first).  It also needs the reverse direction
// intact so SkyTraq binary configuration frames can be pushed from the host.
//
// So: a byte-for-byte bridge in both directions, and nothing else.
//
// Board facts, read out of the TinkerNav V25 netlist rather than transcribed
// (kicad-cli sch export netlist on "TinkerNav V25.kicad_sch"):
//
//   net GPS_RX    U3.27 IO47  ->  U2.3  RXD    ESP32 transmits to the receiver
//   net GPS_TX    U3.30 IO48  <-  U2.2  TXD    ESP32 receives from the receiver
//   net GPS_RXD2  U3.20 IO16  ->  U2.15 RXD2   RTCM-in, unused here
//   net NEOPIX    U3.26 IO26  ->  L1.3  DI     status LED
//   U3 = ESP32-S3-MINI-1-N8, U2 = PX1105R, J7 USB-C wired to ESP32 D+/D- (native
//   USB CDC -- there is no bridge chip, so `Serial` IS the USB port).
//
// Note the naming trap: TinkerNav's config.h calls IO47 "GNSS_TX" and IO48
// "GNSS_RX", named from the ESP32's point of view, and then passes them to
// Serial1.begin(baud, cfg, rxPin, txPin) as (GNSS_RX, GNSS_TX).  The pins below
// are named for the ESP32 UART role to keep that straight.
//
// Baud: autodetected at boot by scoring valid NMEA checksums at each candidate
// rate, because a receiver left reconfigured by earlier bench work otherwise
// presents as line noise and reads like a dead board.  The lock is rechecked
// continuously and re-run after a silence window, so a configuration frame that
// changes the receiver's baud mid-session is recovered from rather than ending
// the session.  A COCOM block does *not* trigger that path: a withheld fix
// still emits well-formed GGA/RMC with empty fields, so the checksum score
// stays high and the lock holds.
//
// Everything this sketch prints for humans is prefixed with '#', which is not a
// legal NMEA start character; gnss_nmea_monitor.py drops those lines.  Nothing
// else is ever injected into the data path.

#include <Arduino.h>

// ---- Pins (ESP32 UART role, see netlist note above) ----
//
// Defaults are per-chip because the pin numbers are NOT portable: the TinkerNav
// V25 map below is IO47/IO48, and those GPIOs do not exist on an ESP32-C3, which
// stops at GPIO21.  Flashing the S3 map to a C3 gets you a silent board and a
// hunt for a wiring fault that was never there.
//
// Override for hand-wired boards without editing this file:
//   arduino-cli compile --build-property \
//     "compiler.cpp.extra_flags=-DGNSS_UART_RX_PIN=4 -DGNSS_UART_TX_PIN=5" ...
//
// Whether the TX pin was asserted by whoever built this, captured before the
// defaults below fill it in.  It gates whether this firmware is ever willing to
// *drive* a pin -- see kTxTrusted.
#if defined(GNSS_UART_TX_PIN)
  #define GNSS_TX_PIN_SUPPLIED 1
#else
  #define GNSS_TX_PIN_SUPPLIED 0
#endif

#if !defined(GNSS_UART_RX_PIN)
  #if defined(ARDUINO_ARCH_RP2040)
    // TinkerNav dual-MCU boards (RP2040 + ESP32-C3 + PX1125R).  Serial1 on its
    // default pins, per Tinkerbug's own rover firmware:
    //   "GNSS input/output Serial is Serial1 using default 0,1 (TX, RX) pins"
    //   (TinkerRTKLoRa/RP2040_Rover_LoRaRadio).  Serial2 on 4/5 carries RTCM
    //   into the receiver's RXD2 and is not used here.
    #define GNSS_UART_RX_PIN 1
  #elif CONFIG_IDF_TARGET_ESP32S3
    #define GNSS_UART_RX_PIN 48   // TinkerNav V25 net GPS_TX, PX1105R TXD
  #elif CONFIG_IDF_TARGET_ESP32C3
    // Guess.  On the dual-MCU boards the receiver is wired to the RP2040, not
    // the C3, so a C3 build is expected to find nothing -- see README.
    #define GNSS_UART_RX_PIN 4
  #else
    #define GNSS_UART_RX_PIN 16
  #endif
#endif

#if !defined(GNSS_UART_TX_PIN)
  #if defined(ARDUINO_ARCH_RP2040)
    #define GNSS_UART_TX_PIN 0    // Serial1 default TX -> receiver RXD
  #elif CONFIG_IDF_TARGET_ESP32S3
    #define GNSS_UART_TX_PIN 47   // TinkerNav V25 net GPS_RX, PX1105R RXD
  #elif CONFIG_IDF_TARGET_ESP32C3
    #define GNSS_UART_TX_PIN 5
  #else
    #define GNSS_UART_TX_PIN 17
  #endif
#endif

// Is the TX pin known, or guessed?  Driving a guessed pin is the one thing here
// that can damage hardware: if it happens to be another device's output, the two
// drivers fight.  Reading is always safe, so an unverified map gets a listen-only
// bridge rather than a coin flip.
//
// Trusted when the builder supplied it explicitly, or on the S3, where the
// default came out of the TinkerNav V25 netlist rather than a guess.
// RP2040 counts as trusted: its map comes from Tinkerbug's shipping firmware
// for this very board, which is as authoritative as a netlist.
#if GNSS_TX_PIN_SUPPLIED || CONFIG_IDF_TARGET_ESP32S3 || defined(ARDUINO_ARCH_RP2040)
static constexpr bool kTxTrusted = true;
#else
static constexpr bool kTxTrusted = false;
#endif

static constexpr int PIN_GNSS_UART_RX = GNSS_UART_RX_PIN;
static constexpr int PIN_GNSS_UART_TX = GNSS_UART_TX_PIN;

// Status LED.  -1 disables it; the V25's NeoPixel is on a GPIO that only exists
// on the S3, so anything else stays dark rather than driving a pin at random.
#if defined(GNSS_LED_PIN)
static constexpr int PIN_NEOPIX = GNSS_LED_PIN;
#elif CONFIG_IDF_TARGET_ESP32S3
static constexpr int PIN_NEOPIX = 26;   // net NEOPIX
#elif defined(RGB_BUILTIN)
static constexpr int PIN_NEOPIX = RGB_BUILTIN;
#else
static constexpr int PIN_NEOPIX = -1;
#endif

// ---- Baud detection ----
// PX1105R ships at 115200 and TinkerNav keeps it there, so that is tried first
// and wins ties; the rest cover a receiver left reconfigured by earlier work.
static const uint32_t kBaudCandidates[] = {115200, 9600, 38400, 57600,
                                           19200,  4800, 230400, 460800, 921600};
static constexpr size_t kNumCandidates = sizeof(kBaudCandidates) / sizeof(kBaudCandidates[0]);

static constexpr uint32_t kProbeWindowMs   = 500;   // listen per candidate
static constexpr int      kProbeGoodEnough = 3;     // valid sentences => stop early
static constexpr uint32_t kRelockSilenceMs = 5000;  // no valid NMEA => re-probe

static uint32_t g_baud = 0;

// Live UART pins.  Start at the compiled-in map; the boot scan may replace the
// RX pin on a board whose wiring is not known ahead of time.  TX stays where it
// was configured -- it is never guessed, only ever taken from a schematic.
static int g_rxPin = PIN_GNSS_UART_RX;
static int g_txPin = kTxTrusted ? PIN_GNSS_UART_TX : -1;   // -1 = never drive

// Open UART1 on the given pins.  The two cores disagree about how pins are
// assigned -- ESP32 takes them in begin(), RP2040 wants setRX/setTX first --
// and every caller here needs "reopen on these pins", so the difference is
// confined to this one function.  tx < 0 means listen-only.
static void gnssUartBegin(uint32_t baud, int rx, int tx)
{
    Serial1.end();
    delay(20);
#if defined(ARDUINO_ARCH_RP2040)
    // The RP2040's UARTs reach only a fixed set of pins; setRX/setTX reject
    // anything else, which is what keeps the scan below honest.
    Serial1.setRX((pin_size_t)rx);
    if (tx >= 0) Serial1.setTX((pin_size_t)tx);
    Serial1.begin(baud, SERIAL_8N1);
#else
    Serial1.begin(baud, SERIAL_8N1, rx, tx);
#endif
}

// ---- Status LED ----
// neopixelWrite() is provided by the esp32 core (esp32-hal-rgb-led.h, pulled in
// by Arduino.h).  Guarded so the sketch still builds on a core without it.
#if !defined(ARDUINO_ARCH_RP2040) && __has_include(<esp32-hal-rgb-led.h>)
#define HAVE_NEOPIXEL 1
#else
#define HAVE_NEOPIXEL 0
#endif

static void setLed(uint8_t r, uint8_t g, uint8_t b)
{
#if HAVE_NEOPIXEL
    if (PIN_NEOPIX >= 0) neopixelWrite(PIN_NEOPIX, r, g, b);
#else
    (void)r; (void)g; (void)b;
#endif
}

// ---- NMEA checksum scoring -------------------------------------------------
//
// A sentence is "$" .. "*HH", where HH is the XOR of everything between the
// delimiters.  Scoring on the checksum rather than on the mere presence of '$'
// matters: at the wrong baud a stream is full of coincidental '$' bytes, and
// only the checksum reliably separates a real lock from framing garbage.

class NmeaScorer
{
public:
    void reset() { state_ = kIdle; xor_ = 0; hi_ = 0; valid_ = 0; }

    int valid() const { return valid_; }

    // Returns true when this byte completed a checksum-valid sentence.
    bool feed(uint8_t b)
    {
        switch (state_)
        {
            case kIdle:
                if (b == '$') { state_ = kBody; xor_ = 0; }
                break;

            case kBody:
                if (b == '*')      { state_ = kSumHi; }
                else if (b == '$') { xor_ = 0; }              // restart on a new '$'
                else if (b == '\r' || b == '\n') { state_ = kIdle; }
                else               { xor_ ^= b; }
                break;

            case kSumHi:
                if (!isHex(b)) { state_ = kIdle; break; }
                hi_ = hexVal(b);
                state_ = kSumLo;
                break;

            case kSumLo:
            {
                state_ = kIdle;
                if (!isHex(b)) break;
                const uint8_t got = (uint8_t)((hi_ << 4) | hexVal(b));
                if (got == xor_) { valid_++; return true; }
                break;
            }
        }
        return false;
    }

private:
    static bool isHex(uint8_t c)
    {
        return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
    }
    static uint8_t hexVal(uint8_t c)
    {
        if (c <= '9') return (uint8_t)(c - '0');
        return (uint8_t)((c | 0x20) - 'a' + 10);
    }

    enum State : uint8_t { kIdle, kBody, kSumHi, kSumLo };
    State   state_ = kIdle;
    uint8_t xor_   = 0;
    uint8_t hi_    = 0;
    int     valid_ = 0;
};

static NmeaScorer g_scorer;

// ---- Baud probe ------------------------------------------------------------

static int probeBaud(uint32_t baud)
{
    // Listen-only while probing: nothing is driven until a pin map is confirmed.
    gnssUartBegin(baud, g_rxPin, -1);

    // Discard the first partial sentence and any framing junk from the switch.
    delay(60);
    while (Serial1.available()) Serial1.read();

    NmeaScorer probe;
    const uint32_t deadline = millis() + kProbeWindowMs;
    while ((int32_t)(millis() - deadline) < 0)
    {
        while (Serial1.available())
        {
            probe.feed((uint8_t)Serial1.read());
            if (probe.valid() >= kProbeGoodEnough) return probe.valid();
        }
        delay(1);
    }
    return probe.valid();
}

static uint32_t detectBaud()
{
    uint32_t best      = 0;
    int      bestScore = 0;

    for (size_t i = 0; i < kNumCandidates; i++)
    {
        setLed(0, 0, 24);  // blue: probing
        const int score = probeBaud(kBaudCandidates[i]);
        Serial.printf("#   probe %7lu -> %d valid sentence(s)\n",
                      (unsigned long)kBaudCandidates[i], score);
        if (score > bestScore)
        {
            bestScore = score;
            best      = kBaudCandidates[i];
            if (bestScore >= kProbeGoodEnough) break;  // first candidate wins ties
        }
    }

    return best;   // 0 means nothing parsed at any rate
}

static void openGnss(uint32_t baud)
{
    gnssUartBegin(baud, g_rxPin, g_txPin);
    g_scorer.reset();
}

// ---- Pin scan --------------------------------------------------------------
//
// For a board whose schematic is not to hand, the board itself is the most
// reliable source of its own wiring.  Bind UART1's RX to each candidate GPIO in
// turn and score NMEA checksums; the pin carrying the receiver announces itself.
//
// This is deliberately RX-ONLY -- TX is bound to -1 throughout.  Driving an
// unknown pin as a UART transmitter could fight whatever else is connected to
// it, so the scan only ever listens.  That is enough to *read* NMEA, which is
// what #491 needs; the TX pin only matters for pushing configuration frames and
// is better learned from a schematic than guessed at.
//
// Pin sets exclude what cannot be a UART input on each part: the embedded-flash
// pins, the native-USB pair carrying this very console, and (on the S3) the
// octal-PSRAM range.

#if defined(ARDUINO_ARCH_RP2040)
// The RP2040's UARTs are not freely routable: UART0 (Serial1) reaches RX on
// only GPIO 1, 13, 17 and 29.  Scanning anything else is not "no receiver
// there", it is "this UART cannot see that pin", so the set is exactly the
// reachable ones.  GPIO1 is the expected answer on the TinkerNav boards.
static const int kScanPins[] = {1, 13, 17};
#elif CONFIG_IDF_TARGET_ESP32C3
// C3 has GPIO0-21.  11-17 are embedded SPI flash, 18/19 are native USB.
static const int kScanPins[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 21};
#elif CONFIG_IDF_TARGET_ESP32S3
// S3: 26-32 SPI flash, 33-37 octal PSRAM, 19/20 native USB.
static const int kScanPins[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
                                15, 16, 17, 18, 21, 38, 39, 40, 41, 42, 47, 48};
#else
static const int kScanPins[] = {16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33};
#endif
static constexpr size_t kNumScanPins = sizeof(kScanPins) / sizeof(kScanPins[0]);

// Listen on one pin at one rate.  Returns valid sentences; reports raw byte
// count too, because "bytes but no sentences" (wrong baud) and "no bytes at
// all" (nothing wired here) are different faults and want different fixes.
static int probePin(int pin, uint32_t baud, int& bytesSeen)
{
    gnssUartBegin(baud, pin, -1);
    delay(50);
    while (Serial1.available()) Serial1.read();

    NmeaScorer probe;
    bytesSeen = 0;
    const uint32_t deadline = millis() + 350;
    while ((int32_t)(millis() - deadline) < 0)
    {
        while (Serial1.available())
        {
            bytesSeen++;
            probe.feed((uint8_t)Serial1.read());
            if (probe.valid() >= 2) return probe.valid();
        }
        delay(1);
    }
    return probe.valid();
}

// Tri-state each candidate pin to tell "nothing is powered" from "powered but
// not talking".  Read with the internal pulldown, then the pullup: a pin that
// holds its level against both is being driven by something, while one that
// simply follows the resistor is floating.
//
// Still input-only, so still safe on an unknown board.  This matters after a
// scan finds zero bytes everywhere, because an idle UART transmitter sits HIGH
// -- so a driven-high pin means a live device that just is not sending, whereas
// all-floating means nothing on the other end is powered at all.
static void reportPinLevels()
{
    Serial.println("# pin levels (input-only, no pin is driven):");
    int driven = 0;
    for (size_t i = 0; i < kNumScanPins; i++)
    {
        const int p = kScanPins[i];
        pinMode(p, INPUT_PULLDOWN);
        delay(3);
        const int withPd = digitalRead(p);
        pinMode(p, INPUT_PULLUP);
        delay(3);
        const int withPu = digitalRead(p);
        pinMode(p, INPUT);

        const char* verdict;
        if (withPd == 1 && withPu == 1)      { verdict = "driven HIGH  <- live (idle UART TX looks like this)"; driven++; }
        else if (withPd == 0 && withPu == 0) { verdict = "driven LOW   <- something is holding it down"; driven++; }
        else                                 { verdict = "floating"; }
        Serial.printf("#   IO%-2d %s\n", p, verdict);
    }

    if (driven == 0)
    {
        Serial.println("# every candidate pin floats: nothing on the other end is powered.");
        Serial.println("# most likely the receiver's rail is not up -- many carrier boards");
        Serial.println("# need external/battery power, or an enable line, for the GNSS rail.");
    }
    else
    {
        Serial.printf("# %d pin(s) driven: something IS alive and simply not emitting NMEA.\n",
                      driven);
        Serial.println("# suspect the module is held in reset, or its output is disabled.");
    }
}

static bool scanPins(int& foundPin, uint32_t& foundBaud)
{
    Serial.println("# scanning GPIOs for a receiver (RX-only, nothing is driven)...");

    // Pass 1: every candidate at the SkyTraq default rate.  Also records which
    // pins carry *any* traffic, which narrows pass 2 to somewhere worth looking.
    bool active[kNumScanPins] = {false};
    for (size_t i = 0; i < kNumScanPins; i++)
    {
        setLed(0, 0, 24);
        int bytes = 0;
        const int valid = probePin(kScanPins[i], 115200, bytes);
        if (bytes) active[i] = true;
        if (valid > 0)
        {
            Serial.printf("#   IO%-2d @ 115200 -> %d sentence(s)  << FOUND\n",
                          kScanPins[i], valid);
            foundPin  = kScanPins[i];
            foundBaud = 115200;
            return true;
        }
        Serial.printf("#   IO%-2d @ 115200 -> %d bytes, no valid NMEA\n",
                      kScanPins[i], bytes);
    }

    // Pass 2: pins that were noisy but unparseable are probably the right wire
    // at the wrong rate.
    for (size_t i = 0; i < kNumScanPins; i++)
    {
        if (!active[i]) continue;
        for (size_t b = 0; b < kNumCandidates; b++)
        {
            if (kBaudCandidates[b] == 115200) continue;
            int bytes = 0;
            const int valid = probePin(kScanPins[i], kBaudCandidates[b], bytes);
            if (valid > 0)
            {
                Serial.printf("#   IO%-2d @ %lu -> %d sentence(s)  << FOUND\n",
                              kScanPins[i], (unsigned long)kBaudCandidates[b], valid);
                foundPin  = kScanPins[i];
                foundBaud = kBaudCandidates[b];
                return true;
            }
        }
        Serial.printf("#   IO%-2d had traffic but no valid NMEA at any rate\n",
                      kScanPins[i]);
    }
    return false;
}

// ---- Setup / loop ----------------------------------------------------------

void setup()
{
    Serial.begin(115200);

    // Native USB CDC: wait briefly for the host to attach so the banner is not
    // lost, but never block -- the bridge has to run headless too.
    const uint32_t deadline = millis() + 2000;
    while (!Serial && (int32_t)(millis() - deadline) < 0) delay(10);
    delay(200);

    setLed(0, 0, 24);
    Serial.println();
    Serial.println("# GNSS passthrough (SkyTraq PX1105R) -- issue #491 bench rig");
    // Print the pins actually compiled in.  A board wired differently from the
    // chip default is the most likely reason for silence, and it is invisible
    // unless the firmware says which pins it is listening on.
#if defined(ARDUINO_ARCH_RP2040)
    const char* chip = "RP2040";
#else
    const char* chip = ESP.getChipModel();
#endif
    Serial.printf("# chip %s   UART1 RX=GPIO%d (<-receiver TXD)\n",
                  chip, PIN_GNSS_UART_RX);
    if (kTxTrusted)
    {
        Serial.printf("# TX=IO%d (->receiver RXD), from a verified pin map\n",
                      PIN_GNSS_UART_TX);
    }
    else
    {
        Serial.println("# TX unbound -- this pin map is a guess, so nothing is driven.");
        Serial.println("# Reading NMEA works. To send config frames, rebuild with an");
        Serial.println("# explicit -DGNSS_UART_TX_PIN=<pin> taken from the schematic.");
    }
    Serial.println("# detecting receiver baud on the configured pin...");

    g_baud = detectBaud();

    if (g_baud == 0)
    {
        // Nothing on the configured pin.  On a board whose pin map came from a
        // #define rather than a schematic that is the expected first result, so
        // fall through to asking the hardware instead of reporting a fault.
        Serial.println("# nothing on the configured RX pin at any rate.");
        int      pin  = -1;
        uint32_t baud = 0;
        if (scanPins(pin, baud))
        {
            g_rxPin = pin;
            g_baud  = baud;
            Serial.printf("# receiver found on IO%d @ %lu baud.\n",
                          pin, (unsigned long)baud);
            // The scan relocated RX, so any compiled TX pin was part of the same
            // wrong guess.  Drop it rather than drive a pin on that evidence.
            if (g_txPin >= 0)
            {
                Serial.printf("# dropping TX (was IO%d): the pin map it came from was wrong.\n",
                              g_txPin);
                g_txPin = -1;
            }
            Serial.printf("# rebuild with -DGNSS_UART_RX_PIN=%d to skip the scan next boot,\n", pin);
            Serial.println("# and add -DGNSS_UART_TX_PIN=<pin> when you know the transmit side.");
        }
        else
        {
            Serial.println("# no receiver found on any candidate GPIO.");
            reportPinLevels();
            g_baud = 115200;   // bridge anyway so the USB path is still usable
        }
    }

    openGnss(g_baud);

    Serial.printf("# bridging IO%d @ %lu baud 8N1. All further output is raw receiver data.\n",
                  g_rxPin, (unsigned long)g_baud);
    Serial.println("# (host->device bytes are forwarded verbatim for SkyTraq binary frames)");
}

void loop()
{
    static uint32_t lastValidMs = 0;
    static bool     everValid   = false;
    static uint32_t lastLedMs   = 0;

    if (lastValidMs == 0) lastValidMs = millis();

    // Receiver -> host.  Bulk-moved, never line-buffered, so binary responses
    // (0xA0 0xA1 ... 0x0D 0x0A ACK/NACK frames) survive intact.
    uint8_t buf[256];
    int n = Serial1.available();
    if (n > 0)
    {
        if (n > (int)sizeof(buf)) n = sizeof(buf);
        n = Serial1.readBytes(buf, n);
        if (n > 0)
        {
            Serial.write(buf, n);
            for (int i = 0; i < n; i++)
            {
                if (g_scorer.feed(buf[i]))
                {
                    lastValidMs = millis();
                    everValid   = true;
                }
            }
        }
    }

    // Host -> receiver, verbatim.
    int m = Serial.available();
    if (m > 0)
    {
        if (m > (int)sizeof(buf)) m = sizeof(buf);
        m = Serial.readBytes(buf, m);
        if (m > 0) Serial1.write(buf, m);
    }

    const uint32_t now = millis();

    // Status LED: green = valid NMEA flowing, amber = bytes but nothing valid,
    // red = silent.  Updated at 5 Hz so it does not compete with the data path.
    if (now - lastLedMs >= 200)
    {
        lastLedMs = now;
        const uint32_t sinceValid = now - lastValidMs;
        if (sinceValid < 1500)      setLed(0, 20, 0);
        else if (everValid)         setLed(24, 12, 0);
        else                        setLed(24, 0, 0);
    }

    // Re-lock only after a genuine silence window, and only if there was a lock
    // to lose.  The `everValid` guard matters more than it looks: a receiver
    // that has never emitted NMEA is usually one that needs *configuring*, and
    // re-probing tears the UART down and back up every few seconds, which eats
    // the ACKs those configuration commands are waiting on.  Without this the
    // bridge fights the very tool trying to fix the receiver.
    //
    // A COCOM block keeps emitting valid sentences with empty fields, so this
    // does not fire during a test.
    if (everValid && now - lastValidMs > kRelockSilenceMs)
    {
        Serial.println();
        Serial.printf("# no valid NMEA for %lu ms on IO%d @ %lu baud; re-detecting...\n",
                      (unsigned long)(now - lastValidMs), g_rxPin, (unsigned long)g_baud);
        const uint32_t again = detectBaud();
        g_baud = (again != 0) ? again : g_baud;   // keep the last known rate
        openGnss(g_baud);
        Serial.printf("# re-locked on IO%d @ %lu baud 8N1.\n", g_rxPin, (unsigned long)g_baud);
        lastValidMs = millis();
    }
}
