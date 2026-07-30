package com.tinkerbug.tinkerrocket.session

import java.io.File
import kotlin.test.Test

class TempRealJsonTest {
    @Test
    fun realApiResponse() {
        val body = File(
            "/private/tmp/claude-501/-Users-christianpedersen-Documents-Hobbies-" +
                "ModelRockets-Code--claude-worktrees-pre-flight-code-review-545b89/" +
                "4eada5b7-e007-4512-a1d0-d684b374246c/scratchpad/releases.json",
        ).readText()
        println("PARSED=" + UpdateCheck.parseReleases(body))
        println("DECISION=" + UpdateCheck.updateAvailable("0.0.1", body))
    }
}
