// Copyright (c) Choreo contributors
package choreo.trajectory

import choreo.util.AllianceFlipUtil.Flipper
import choreo.util.AllianceFlipUtil.flipX
import choreo.util.AllianceFlipUtil.flipY
import choreo.util.AllianceFlipUtil.flipper
import edu.wpi.first.math.MathUtil
import edu.wpi.first.struct.Struct
import net.unnamedrobotics.lib.math.Pose
import net.unnamedrobotics.lib.math.interpolate
import java.nio.ByteBuffer
import java.util.Arrays

/** A single swerve robot sample in a Trajectory.  */
class SwerveSample
/**
 * Constructs a SwerveSample with the specified parameters.
 *
 * @param t The timestamp of this sample, relative to the beginning of the trajectory.
 * @param x The X position of the sample in meters.
 * @param y The Y position of the sample in meters.
 * @param heading The heading of the sample in radians, with 0 being in the +X direction.
 * @param vx The velocity of the sample in the X direction in m/s.
 * @param vy The velocity of the sample in the Y direction in m/s.
 * @param omega The angular velocity of the sample in rad/s.
 * @param ax The acceleration of the sample in the X direction in m/s².
 * @param ay The acceleration of the sample in the Y direction in m/s².
 * @param alpha The angular acceleration of the sample in rad/s².
 * @param moduleForcesX The force on each swerve module in the X direction in Newtons. Module
 * forces appear in the following order: [FL, FR, BL, BR].
 * @param moduleForcesY The force on each swerve module in the Y direction in Newtons. Module
 * forces appear in the following order: [FL, FR, BL, BR].
 */(
    /** The timestamp of this sample, relative to the beginning of the trajectory.  */
    val t: Double,
    /** The X position of the sample relative to the blue alliance wall origin in meters.  */
    val x: Double,
    /** The Y position of the sample relative to the blue alliance wall origin in meters.  */
    val y: Double,
    /** The heading of the sample in radians, with 0 being in the +X direction.  */
    val heading: Double,
    /** The velocity of the sample in the X direction in m/s.  */
    val vx: Double,
    /** The velocity of the sample in the Y direction in m/s.  */
    val vy: Double,
    /** The angular velocity of the sample in rad/s.  */
    val omega: Double,
    /** The acceleration of the in the X direction in m/s².  */
    val ax: Double,
    /** The acceleration of the in the Y direction in m/s².  */
    val ay: Double,
    /** The angular acceleration of the sample in rad/s².  */
    val alpha: Double,
    /**
     * The force on each swerve module in the X direction in Newtons. Module forces appear in the
     * following order: [FL, FR, BL, BR].
     */
    private val fx: DoubleArray?,
    /**
     * The force on each swerve module in the Y direction in Newtons Module forces appear in the
     * following order: [FL, FR, BL, BR].
     */
    private val fy: DoubleArray?
) : TrajectorySample<SwerveSample> {
    /**
     * A null safe getter for the module forces in the X direction.
     *
     * @return The module forces in the X direction.
     */
    fun moduleForcesX(): DoubleArray {
        if (fx == null || fx.size != 4) {
            return EMPTY_MODULE_FORCES
        }
        return fx
    }

    /**
     * A null safe getter for the module forces in the Y direction.
     *
     * @return The module forces in the Y direction.
     */
    fun moduleForcesY(): DoubleArray {
        if (fy == null || fy.size != 4) {
            return EMPTY_MODULE_FORCES
        }
        return fy
    }

    override fun getTimestamp(): Double {
        return t
    }

    override fun getPose(): Pose {
        return Pose(x, y, heading)
    }

    override fun getChassisSpeeds(): Pose {
        return Pose(vx, vy, omega)
    }

    override fun interpolate(endValue: SwerveSample, timestamp: Double): SwerveSample {
        val scale = (timestamp - this.t) / (endValue.t - this.t)
        val tmp = pose.lerp(endValue.pose, scale)
        val interp_pose = Pose(tmp.x, tmp.y, interpolate(pose.angle, endValue.pose.angle, scale))

        val interp_fx = DoubleArray(4)
        val interp_fy = DoubleArray(4)
        for (i in 0..3) {
            interp_fx[i] =
                MathUtil.interpolate(moduleForcesX()[i], endValue.moduleForcesX()[i], scale)
            interp_fy[i] =
                MathUtil.interpolate(moduleForcesY()[i], endValue.moduleForcesY()[i], scale)
        }

        return SwerveSample(
            MathUtil.interpolate(this.t, endValue.t, scale),
            interp_pose.x,
            interp_pose.y,
            interp_pose.angle,
            MathUtil.interpolate(this.vx, endValue.vx, scale),
            MathUtil.interpolate(this.vy, endValue.vy, scale),
            MathUtil.interpolate(this.omega, endValue.omega, scale),
            MathUtil.interpolate(this.ax, endValue.ax, scale),
            MathUtil.interpolate(this.ay, endValue.ay, scale),
            MathUtil.interpolate(this.alpha, endValue.alpha, scale),
            interp_fx,
            interp_fy
        )
    }

    override fun offsetBy(timestampOffset: Double): SwerveSample {
        return SwerveSample(
            this.t + timestampOffset,
            this.x,
            this.y,
            this.heading,
            this.vx,
            this.vy,
            this.omega,
            this.ax,
            this.ay,
            this.alpha,
            this.moduleForcesX(),
            this.moduleForcesY()
        )
    }

    override fun flipped(): SwerveSample {
        return when (flipper) {
            Flipper.MIRRORED -> SwerveSample(
                this.t,
                flipX(this.x),
                this.y,
                Math.PI - this.heading,
                -this.vx,
                this.vy,
                -this.omega,
                -this.ax,
                this.ay,
                -this.alpha,  // FL, FR, BL, BR
                // Mirrored
                // -FR, -FL, -BR, -BL
                doubleArrayOf(
                    -moduleForcesX()[1],
                    -moduleForcesX()[0],
                    -moduleForcesX()[3],
                    -moduleForcesX()[2]
                ),  // FL, FR, BL, BR
                // Mirrored
                // FR, FL, BR, BL
                doubleArrayOf(
                    moduleForcesY()[1],
                    moduleForcesY()[0],
                    moduleForcesY()[3],
                    moduleForcesY()[2]
                )
            )

            Flipper.ROTATE_AROUND -> SwerveSample(
                this.t,
                flipX(this.x),
                flipY(this.y),
                Math.PI - this.heading,
                -this.vx,
                -this.vy,
                -this.omega,
                -this.ax,
                -this.ay,
                -this.alpha,
                Arrays.stream(this.moduleForcesX()).map { x: Double -> -x }
                    .toArray(),
                Arrays.stream(this.moduleForcesY()).map { y: Double -> -y }
                    .toArray())

            else -> throw IllegalArgumentException()
        }
    }

    override fun makeArray(length: Int): Array<SwerveSample?> {
        return arrayOfNulls(length)
    }

    private class SwerveSampleStruct : Struct<SwerveSample> {
        override fun getTypeClass(): Class<SwerveSample> {
            return SwerveSample::class.java
        }

        override fun getTypeName(): String {
            return "SwerveSample"
        }

        override fun getSize(): Int {
            return Struct.kSizeDouble * 18
        }

        override fun getSchema(): String {
            return ("double timestamp;"
                    + "Pose2d pose;"
                    + "double vx;"
                    + "double vy;"
                    + "double omega;"
                    + "double ax;"
                    + "double ay;"
                    + "double alpha;"
                    + "double moduleForcesX[4];"
                    + "double moduleForcesY[4];")
        }

        override fun unpack(bb: ByteBuffer): SwerveSample {
            return SwerveSample(
                bb.getDouble(),
                bb.getDouble(),
                bb.getDouble(),
                bb.getDouble(),
                bb.getDouble(),
                bb.getDouble(),
                bb.getDouble(),
                bb.getDouble(),
                bb.getDouble(),
                bb.getDouble(),
                doubleArrayOf(bb.getDouble(), bb.getDouble(), bb.getDouble(), bb.getDouble()),
                doubleArrayOf(bb.getDouble(), bb.getDouble(), bb.getDouble(), bb.getDouble())
            )
        }

        override fun pack(bb: ByteBuffer, value: SwerveSample) {
            bb.putDouble(value.t)
            bb.putDouble(value.x)
            bb.putDouble(value.y)
            bb.putDouble(value.heading)
            bb.putDouble(value.vx)
            bb.putDouble(value.vy)
            bb.putDouble(value.omega)
            bb.putDouble(value.ax)
            bb.putDouble(value.ay)
            bb.putDouble(value.alpha)
            for (i in 0..3) {
                bb.putDouble(value.moduleForcesX()[i])
            }
            for (i in 0..3) {
                bb.putDouble(value.moduleForcesY()[i])
            }
        }
    }

    override fun equals(obj: Any?): Boolean {
        if (obj !is SwerveSample) {
            return false
        }

        val other = obj
        return this.t == other.t && (this.x == other.x
                ) && (this.y == other.y
                ) && (this.heading == other.heading
                ) && (this.vx == other.vx
                ) && (this.vy == other.vy
                ) && (this.omega == other.omega
                ) && (this.ax == other.ax
                ) && (this.ay == other.ay
                ) && (this.alpha == other.alpha
                ) && fx.contentEquals(other.fx) && fy.contentEquals(other.fy)
    }

    companion object {
        private val EMPTY_MODULE_FORCES = doubleArrayOf(0.0, 0.0, 0.0, 0.0)

        /** The struct for the SwerveSample class.  */
        val struct: Struct<SwerveSample> = SwerveSampleStruct()
    }
}
