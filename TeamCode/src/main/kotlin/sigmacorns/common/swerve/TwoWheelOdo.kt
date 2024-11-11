package sigmacorns.common.swerve

import com.qualcomm.robotcore.hardware.IMU
import net.unnamedrobotics.lib.hardware.interfaces.Encoder
import net.unnamedrobotics.lib.math.AngleUnit
import net.unnamedrobotics.lib.math.Pose
import net.unnamedrobotics.lib.math.Vector2
import net.unnamedrobotics.lib.math.radians
import net.unnamedrobotics.lib.rerun.RerunConnection
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

class TwoWheelOdo(val xEncoder: Encoder, val yEncoder: Encoder, val imu: IMU, var pose: Pose) {
    val R: Double = 0.01
    val LOG_WINDOW_LEN = 100
    val TICKS_PER_REV = 2048

    var tickOld = Vector2(xEncoder.getCounts(),yEncoder.getCounts())
    var thetaOld = imu.robotYawPitchRollAngles.yaw

    fun update() {
        val ticks = Vector2(xEncoder.getCounts(),yEncoder.getCounts())
        val dist = (ticks-tickOld)/TICKS_PER_REV*2*PI*R
        val yaw = imu.robotYawPitchRollAngles.yaw
        val theta = yaw
        val dTheta = theta-thetaOld

        val dxRobot = sin(dTheta)/(dTheta) * dist.x - (1 - cos(dTheta))/dTheta*dist.y
        val dyRobot = (1- cos(dTheta))/dTheta*dist.x + sin(dTheta)/(dTheta) * dist.y
        val d = Vector2(dxRobot,dyRobot).rotate(yaw.radians)

        pose += Pose(d.x,d.y,dTheta)
        tickOld = ticks
        thetaOld = theta
    }

    private val window: ArrayList<Vector2> = arrayListOf()
    private var windowPtr = 0
    fun rerun(connection: RerunConnection) {
        if(window.size< LOG_WINDOW_LEN) window.add(pose) else window[windowPtr] = pose
        windowPtr = (windowPtr+1) % LOG_WINDOW_LEN
//        connection.log("localization/pos", window)
//        connection.log("localization/heading", pose.angle)
    }

    fun telemetry(telemetry: Telemetry) {
        telemetry.addData("pose", pose)
    }
}