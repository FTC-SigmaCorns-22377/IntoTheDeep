package sigmacorns.constants

enum class TiltPositions(val x: Double) {
    STRAIGHT(0.65),
    UP(0.0),
    DOWN(1.0),
    AUTO(0.45);

    fun next() = when(this) {
        STRAIGHT -> DOWN
        UP -> STRAIGHT
        DOWN -> STRAIGHT
        AUTO -> STRAIGHT
    }
}