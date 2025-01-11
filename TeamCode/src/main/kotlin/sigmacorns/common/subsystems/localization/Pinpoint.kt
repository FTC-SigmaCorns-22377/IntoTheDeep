//package sigmacorns.common.subsystems.localization
//
//import com.acmerobotics.dashboard.FtcDashboard
//import com.acmerobotics.dashboard.canvas.Canvas
//import com.acmerobotics.dashboard.config.Config
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket
//import eu.sirotin.kotunil.base.m
//import eu.sirotin.kotunil.derived.rad
//import net.unnamedrobotics.lib.driver.gobilda.GoBildaPinpointDriver
//import net.unnamedrobotics.lib.math2.Transform2D
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
//import kotlin.math.cos
//import kotlin.math.sin
//
//@Config
//object PinpointConfig {
//    @JvmField var yawScalar = 1
//    @JvmField var flipX: Boolean = true
//    @JvmField var flipY: Boolean = false
//    @JvmField var xOffset: Double = -100.0
//    @JvmField var yOffset: Double = -143.3
//}
//
//class PinpointLocalizer(val device: GoBildaPinpointDriver) {
//    private var lastSDKPose = Transform2D(0.m, 0.m, 0.rad)
//
//    val global = PinpointConfig
//
//    init {
//        device.initialize()
//        device.recalibrateIMU()
//
//        device.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
//        device.setEncoderDirections(
//            if (global.flipX) GoBildaPinpointDriver.EncoderDirection.REVERSED else GoBildaPinpointDriver.EncoderDirection.FORWARD,
//            if (global.flipY) GoBildaPinpointDriver.EncoderDirection.REVERSED else GoBildaPinpointDriver.EncoderDirection.FORWARD
//        )
//
//        device.setOffsets(global.xOffset, global.yOffset)
//    }
//
//    val isDoneCalibrating: Boolean
//        get() {
//            return device.deviceStatus == GoBildaPinpointDriver.DeviceStatus.READY
//        }
//
//    private fun getEncoderCounts(): Pair<Int, Int> {
//        return Pair(device.encoderX, device.encoderY)
//    }
//
//    private fun getSDKPose(): Transform2D {
//        return lastSDKPose
//    }
//
//    fun update() {
//        device.bulkUpdate()
//
//        if (device.posX.isNaN()) {
//            lastPose = Transform2D(
//                lastSDKPose.getX(DistanceUnit.INCH),
//                lastSDKPose.getY(DistanceUnit.INCH),
//                lastSDKPose.getHeading(AngleUnit.RADIANS)
//            )
//        } else {
//            val tempPose = device.position
//            lastSDKPose = tempPose
//        }
//
//        if (Obi.instance.opModeType == OpModeType.AUTO) {
//            Obi.lastKnownAutoPose = lastPose
//        }
//
//        if (Env.MAP_TO_FIELD) {
//            val packet = TelemetryPacket()
//            drawRobot(packet.field(), getPose())
//            FtcDashboard.getInstance().sendTelemetryPacket(packet)
//        }
//    }
//
//    fun drawRobot(c: Canvas, t: Transform2D) {
//        val ROBOT_RADIUS = 7.0
//
//        c.setStrokeWidth(1)
//        c.strokeCircle(t.x, t.y, ROBOT_RADIUS)
//
//        val rotation = Rotation2D(t.heading)
//        val halfV = Vector2D(rotation.cos, rotation.sin.times(0.5 * ROBOT_RADIUS))
//        val p1 = Vector2D(t.x, t.y).plus(halfV)
//        val p2 = p1.plus(halfV)
//
//        c.strokeLine(p1.x, p1.y, p2.x, p2.y)
//    }
//
//    override fun reset() {
//        device.resetPosAndIMU()
//        lastSDKPose = Transform2D(
//            DistanceUnit.INCH, 0.0, 0.0,
//            AngleUnit.RADIANS, 0.0
//        )
//    }
//
//    override fun write() {
//    }
//}