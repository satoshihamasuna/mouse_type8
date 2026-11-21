package com.example.myapplication.probe

data class RecordingWindowSnapshot(
    val limitUs: Long,
    val firstSourcePtsUs: Long?,
    val lastRelativePtsUs: Long?,
    val writtenFrames: Int,
    val discardedTailFrames: Int,
    val limitReached: Boolean
)

/** Select a real-time prefix. Never synthesize, rescale or fill timestamps. Single drain-thread owner. */
class RecordingWindow(private val limitUs: Long) {
    init { require(limitUs > 0) }
    private var first: Long? = null
    private var previous: Long? = null
    private var relative: Long? = null
    private var written = 0
    private var discarded = 0
    private var reached = false

    fun accept(sourcePtsUs: Long): Boolean {
        previous?.let { check(sourcePtsUs > it) { "PTSが単調増加していません" } }
        previous = sourcePtsUs
        val origin = first ?: sourcePtsUs.also { first = it }
        relative = sourcePtsUs - origin
        if (checkNotNull(relative) >= limitUs) {
            reached = true
            discarded++
            return false
        }
        written++
        return true
    }

    fun snapshot() = RecordingWindowSnapshot(limitUs, first, relative, written, discarded, reached)
}
