package com.tinkerbug.tinkerrocket.app

import android.content.Context
import android.media.AudioAttributes
import android.media.AudioFocusRequest
import android.media.AudioManager
import android.speech.tts.TextToSpeech
import android.speech.tts.UtteranceProgressListener
import com.tinkerbug.tinkerrocket.session.AnnouncerSpeech
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import java.util.Locale
import java.util.concurrent.atomic.AtomicLong

/**
 * Android speech engine behind the [AnnouncerSpeech] seam — the platform half
 * of the iOS `FlightAnnouncer`'s AVSpeechSynthesizer + AVAudioSession.
 *
 * Ducking: iOS sets a `.duckOthers` audio session once; Android has no session,
 * so each utterance requests transient-may-duck audio focus and abandons it
 * when speech ends. Post-Android-8 the OS performs the duck/restore itself.
 *
 * Volume: the speech-assistance usage routes to the media stream, so the
 * hardware volume buttons control callout loudness while one plays — same
 * "phone volume is the volume" behaviour as iOS, no in-app slider.
 */
class TtsSpeaker(context: Context) : AnnouncerSpeech {

    private val audioManager =
        context.getSystemService(Context.AUDIO_SERVICE) as AudioManager

    private val attrs = AudioAttributes.Builder()
        .setUsage(AudioAttributes.USAGE_ASSISTANCE_NAVIGATION_GUIDANCE)
        .setContentType(AudioAttributes.CONTENT_TYPE_SPEECH)
        .build()

    private val focusRequest = AudioFocusRequest
        .Builder(AudioManager.AUDIOFOCUS_GAIN_TRANSIENT_MAY_DUCK)
        .setAudioAttributes(attrs)
        .build()

    /** Engine initialised and usable — drives the toolbar icon colour. */
    private val _ready = MutableStateFlow(false)
    val ready: StateFlow<Boolean> = _ready.asStateFlow()

    /** iOS `lastSessionError` analog (icon turns red). */
    private val _lastError = MutableStateFlow<String?>(null)
    val lastError: StateFlow<String?> = _lastError.asStateFlow()

    /**
     * Set eagerly in [speak], not in onStart: engine callbacks arrive on a
     * binder thread a beat later, and the policy's isBusy check must not see
     * "idle" in that gap and stack a second utterance.
     */
    @Volatile private var busy = false

    /** One utterance queued before engine init — covers the enable-toggle's
     *  "Voice ready" racing a cold TTS engine at first launch. */
    @Volatile private var pending: Pair<String, Boolean>? = null

    private val utteranceSeq = AtomicLong(0)

    private val tts: TextToSpeech = TextToSpeech(context) { status ->
        if (status == TextToSpeech.SUCCESS) {
            onInitialised()
        } else {
            _lastError.value = "TTS init failed (status $status)"
        }
    }

    private fun onInitialised() {
        // en-US to match the iOS voice; fall back silently to the device
        // default if the language pack is missing rather than going mute.
        val r = tts.setLanguage(Locale.US)
        if (r == TextToSpeech.LANG_MISSING_DATA || r == TextToSpeech.LANG_NOT_SUPPORTED) {
            tts.language = Locale.getDefault()
        }
        tts.setAudioAttributes(attrs)
        tts.setOnUtteranceProgressListener(object : UtteranceProgressListener() {
            override fun onStart(utteranceId: String?) {
                busy = true
            }
            override fun onDone(utteranceId: String?) {
                busy = false
                audioManager.abandonAudioFocusRequest(focusRequest)
            }
            @Deprecated("Deprecated in Java")
            override fun onError(utteranceId: String?) {
                busy = false
                audioManager.abandonAudioFocusRequest(focusRequest)
            }
            override fun onError(utteranceId: String?, errorCode: Int) {
                busy = false
                _lastError.value = "TTS error $errorCode"
                audioManager.abandonAudioFocusRequest(focusRequest)
            }
        })
        _ready.value = true
        pending?.let { (text, interrupt) ->
            pending = null
            speak(text, interrupt)
        }
    }

    // ── AnnouncerSpeech ──────────────────────────────────────────────────

    override val isBusy: Boolean get() = busy

    override fun speak(text: String, interrupt: Boolean) {
        if (!_ready.value) {
            // Keep only the newest — a stale callout is worse than none.
            pending = text to interrupt
            return
        }
        // Ducks other audio; the OS restores it on abandon (in onDone).
        audioManager.requestAudioFocus(focusRequest)
        busy = true
        android.util.Log.d(TAG, "Speak: $text")
        // QUEUE_FLUSH in both modes: on interrupt it's the cancel; otherwise
        // the policy has already verified the engine is idle, so there is
        // nothing to flush — and never anything stale to queue behind.
        tts.speak(text, TextToSpeech.QUEUE_FLUSH, null, "tr-${utteranceSeq.incrementAndGet()}")
    }

    override fun stop() {
        tts.stop()
        busy = false
        audioManager.abandonAudioFocusRequest(focusRequest)
    }

    private companion object {
        /** iOS logs "[Announcer]"; same greppable tag for field debugging. */
        const val TAG = "Announcer"
    }
}
