// vals and vars a bit lazy, sorry
// also a lot of these can be vectorized but ugh

package sigmacorns.common.vision

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.IMU
import net.unnamedrobotics.lib.math.Pose
import net.unnamedrobotics.lib.math.radians
import net.unnamedrobotics.lib.math2.Vector2
import net.unnamedrobotics.lib.math2.vec2
import sigmacorns.common.Constants.LIMELIGHT_OFFSET_ROT
import sigmacorns.common.Constants.LIMELIGHT_OFFSET_VEC
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin

class AprilTagLocalizer(val lime: Limelight3A, val imu: IMU) {

    val limeX = LIMELIGHT_OFFSET_VEC[0]
    val limeY = LIMELIGHT_OFFSET_VEC[1]
    val limeZ = LIMELIGHT_OFFSET_VEC[2]
    val yaw = LIMELIGHT_OFFSET_ROT[0]
    val pit = LIMELIGHT_OFFSET_ROT[1]
    val rol = LIMELIGHT_OFFSET_ROT[2]

    val ca = cos(yaw)
    val sa = sin(yaw)
    val cb = cos(pit)
    val sb = sin(pit)
    val cg = cos(rol)
    val sg = sin(rol)

    fun update(): Pose {
        val theta = imu.robotYawPitchRollAngles.yaw.radians // for rotation correction

        val img = lime.latestResult
        if (img == null || !img.isValid) {
            return Pose(22377,22377,22377) // this is the red flag for no data
        }
        val tag = img.fiducialResults.first()
        val tagPose = fieldToTag(tag.fiducialId)

        var robotToTargetSumX = 0.0
        var robotToTargetSumY = 0.0

        for (i in 0..3) {
            var corner = tag.targetCorners[i]
            // corner to cam in image space
            var imgX = corner[0]
            var imgY = corner[1]
            // corner to cam in robot space
            var rotX = ca*cb+(ca*sb*sg-sa*cg)*imgX+(ca*sb*cg+sa*sg)*imgY
            var rotY = sa*cb+(sa*sb*sg+ca*cg)*imgX+(sa*sb*cg-ca*sg)*imgY
            var rotZ = -sb+(cb*sg)*imgX+(cb*cg)*imgY
            // scaling to true height, throw away measured height
            var offset = 2
            if (i >= 2) {
                offset *= -2
            } // TODO check that corners are stored in this order (top to bottom)
            var camToCorX = rotX * ((5.75-limeZ+offset) / rotZ)
            var camToCorY = rotY * ((5.75-limeZ+offset) / rotZ)
            // corner to robot in robot space
            var botToCorX = camToCorX + limeX
            var botToCorY = camToCorY + limeY
            // add world space vector to sum
            robotToTargetSumX += botToCorX * cos(theta.value) - botToCorY * sin(theta.value)
            robotToTargetSumY += botToCorX * sin(theta.value) + botToCorY * cos(theta.value)
        }

        return Pose(tagPose[0]-robotToTargetSumX/4,tagPose[1]-robotToTargetSumY/4,theta)
    }

    fun fieldToTag(id: Int): Array<Double> {
        val x = (-1.0).pow(id)
        val y = (-1.0).pow(id / 3)
        return if (id % 3 == 0) {
            arrayOf(-72 * x,0.0)
        } else {
            arrayOf(48 * x,72 * y)
        }
    }

    // (oops.)

}