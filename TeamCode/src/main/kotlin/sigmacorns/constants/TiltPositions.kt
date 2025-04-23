package sigmacorns.constants

enum class TiltPositions(val x: Double) {
    STRAIGHT(0.55),
    DOWN(1.0);

    fun next() = when(this) {
        STRAIGHT -> DOWN
        DOWN -> STRAIGHT
    }
}