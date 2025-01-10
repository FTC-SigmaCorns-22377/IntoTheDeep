// Copyright (c) Choreo contributors
package choreo.util

import net.unnamedrobotics.lib.math.Angle
import net.unnamedrobotics.lib.math.Pose
import net.unnamedrobotics.lib.math.Vector2
import net.unnamedrobotics.lib.math.radians

/**
 * A utility to standardize flipping of coordinate data based on the current alliance across
 * different years.
 *
 *
 * If every vendor used this, the user would be able to specify the year and no matter the year
 * the vendor's code is from, the user would be able to flip as expected.
 *
 *
 * This API still allows vendors and users to match case against the flipping variant as a way to
 * specially handle cases or throw errors if a variant is explicitly not supported.
 */
object AllianceFlipUtil {
    data class YearInfo(val flipper: Flipper, val fieldLength: Double, val fieldWidth: Double) {}

    // TODO: Update and expand this map
    private val flipperMap: HashMap<Int, YearInfo> = object : HashMap<Int, YearInfo>() {
        init {
            put(2020, YearInfo(Flipper.ROTATE_AROUND, 16.5811, 8.19912))
            put(2021, YearInfo(Flipper.ROTATE_AROUND, 16.5811, 8.19912))
            put(2022, YearInfo(Flipper.ROTATE_AROUND, 16.5811, 8.19912))
            put(2023, YearInfo(Flipper.MIRRORED, 16.5811, 8.19912))
            put(2024, YearInfo(Flipper.MIRRORED, 16.5811, 8.19912))
        }
    }

    private var activeYear: YearInfo = flipperMap[2024]!!

    val flipper: Flipper
        /**
         * Get the flipper that is currently active for flipping coordinates. It's recommended not to
         * store this locally as the flipper may change.
         *
         * @return The active flipper.
         */
        get() = activeYear.flipper

    /**
     * Returns if you are on red alliance.
     *
     * @return If you are on red alliance.
     */
    var shouldFlip: Boolean = false;

    /**
     * Set the year to determine the Alliance Coordinate Flipper to use.
     *
     * @param year The year to set the flipper to. [2020 - 2024]
     */
    fun setYear(year: Int) {
        require(flipperMap.containsKey(year)) { "Year $year is not supported." }
        activeYear = flipperMap[year]!!
    }

    /**
     * Flips the X coordinate.
     *
     * @param x The X coordinate to flip.
     * @return The flipped X coordinate.
     */
    fun flipX(x: Double): Double {
        return activeYear.flipper.flipX(x)
    }

    /**
     * Flips the Y coordinate.
     *
     * @param y The Y coordinate to flip.
     * @return The flipped Y coordinate.
     */
    fun flipY(y: Double): Double {
        return activeYear.flipper.flipY(y)
    }

    /**
     * Flips the heading.
     *
     * @param heading The heading to flip.
     * @return The flipped heading.
     */
    fun flipHeading(heading: Double): Double {
        return activeYear.flipper.flipHeading(heading)
    }

    /**
     * Flips the translation.
     *
     * @param translation The translation to flip.
     * @return The flipped translation.
     */
    fun flip(translation: Vector2): Vector2 {
        return Vector2(flipX(translation.x), flipY(translation.y))
    }

    /**
     * Flips the rotation.
     *
     * @param rotation The rotation to flip.
     * @return The flipped rotation.
     */
    fun flip(rotation: Angle): Angle {
        return when (activeYear.flipper) {
            Flipper.MIRRORED -> rotation.toVector(1.0).let { Vector2(-it.x,it.y).angleFromOrigin }.radians
            Flipper.ROTATE_AROUND -> rotation.toVector(1.0).let { Vector2(-it.x,-it.y).angleFromOrigin }.radians
        }
    }

    /**
     * Flips the pose.
     *
     * @param pose The pose to flip.
     * @return The flipped pose.
     */
    fun flip(pose: Pose): Pose {
        val v = flip(pose as Vector2).toPose(flip(pose.angle.radians).value)
        return Pose(v.x,v.y,flip(pose.angle.radians))
    }

    /** The flipper to use for flipping coordinates.  */
    enum class Flipper {
        /**
         * X becomes fieldLength - x, leaves the y coordinate unchanged, and heading becomes PI -
         * heading.
         */
        MIRRORED {
            override fun flipX(x: Double): Double {
                return activeYear.fieldLength - x
            }

            override fun flipY(y: Double): Double {
                return y
            }

            override fun flipHeading(heading: Double): Double {
                return Math.PI - heading
            }
        },

        /** X becomes fieldLength - x, Y becomes fieldWidth - y, and heading becomes PI - heading.  */
        ROTATE_AROUND {
            override fun flipX(x: Double): Double {
                return activeYear.fieldLength - x
            }

            override fun flipY(y: Double): Double {
                return activeYear.fieldWidth - y
            }

            override fun flipHeading(heading: Double): Double {
                return Math.PI - heading
            }
        };

        /**
         * Flips the X coordinate.
         *
         * @param x The X coordinate to flip.
         * @return The flipped X coordinate.
         */
        abstract fun flipX(x: Double): Double

        /**
         * Flips the Y coordinate.
         *
         * @param y The Y coordinate to flip.
         * @return The flipped Y coordinate.
         */
        abstract fun flipY(y: Double): Double

        /**
         * Flips the heading.
         *
         * @param heading The heading to flip.
         * @return The flipped heading.
         */
        abstract fun flipHeading(heading: Double): Double
    }
}
