package sigmacorns.constants

import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.vec2
import net.unnamedrobotics.lib.math2.vec3

object Vision {
    /**
     * Limelight Pose3d in robot coordinates; ROT contains (yaw, pitch, roll).
     * I avoided vec3 since the components kept returning as expressions and I'm too lazy to fix it.
     * TODO switch back to vec3 when time is less urgent.
     */
    val LIMELIGHT_OFFSET_VEC = arrayOf(0.0,0.0,0.0)
    val LIMELIGHT_OFFSET_ROT = arrayOf(0.0,0.0,0.0)

    // -----Camera Calibration-----
    /**
     * Sigma usually calibrates with OpenCV, but I tried Limelight's default ChArUco.
     * I calibrated at the highest resolution of 1280x960 40fps with full exposure (3300),
     * half sensor gain (22.5/45), and around half red and blue balance (1500/2500).
     * Out of the 100 snapshots I took, 42 were deemed suitable for calibration.
     */
    val REPROJECTION_ERROR = 0.9
    val PRINCIPAL_PIXEL_OFFSET = vec2(7.801,20.489) // default (-2.774,22.549)

    /**
     * Field of View, stored in the formats (HORIZONTAL_TOTAL, LEFT_TO_CENTER, CENTER_TO_RIGHT) and
     * (VERTICAL_TOTAL, TOP_TO_CENTER, CENTER_TO_BOTTOM). Note that the first value is the sum of
     * the second and third values.
     */
    val HORIZONTAL_FOV = vec3(54.621.degrees,27.571.degrees,27.050.degrees) // default (54.505,27.163,27.342)
    val VERTICAL_FOV = vec3(42.448.degrees,22.074.degrees,20.374.degrees) // default (42.239,22.069,20.170)

    /**
     * Distortion coefficients derived from the Brown-Conrady model, stored as (K1,K2,P1,P2,K3)
     * according to FIRST tradition. Here Kn and Pn are respectively the nth radial and tangential
     * distortion coefficients (so K1 is the quadratic coeff., K2 the quartic, K3 the sextic, etc).
     */
    val DISTORTION_COEFFICIENTS = arrayOf(0.215551,-0.770350,-0.000670,0.001687,0.923038)
    // default (0.177168,-0.457341,0.000360,0.002753,0.178259)

    /**
     * Camera matrix, storing focal lengths (fx,fy) at entries (1,1) and (2,2) and optical center
     * (cx,cy) at entries (1,3) and (2,3). Remark that (cx,cy) - [PRINCIPAL_PIXEL_OFFSET] = (1280/2,960/2),
     * the theoretical optical center for the resolution in question (1280x960).
     */
    val CAMERA_MATRIX = arrayOf(arrayOf(1215.838,0.0,647.801),arrayOf(0.0,1215.106,500.489),arrayOf(0.0,0.0,1.0))
    // default ((1221.445,0.0,637.226),(0.0,1223.398,502.549),(0,0,1))
}