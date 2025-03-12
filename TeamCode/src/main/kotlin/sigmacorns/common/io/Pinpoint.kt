package sigmacorns.common.io

import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.mm
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.driver.gobilda.GoBildaPinpointDriver
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.Twist2D
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import sigmacorns.constants.PinpointConfig

class PinpointLocalizer(val device: GoBildaPinpointDriver) {
    private var lastSDKPose = Pose2D(
        DistanceUnit.INCH, 0.0, 0.0,
        AngleUnit.DEGREES, 0.0
    )

    var velocity: Twist2D = Twist2D(0.m/s,0.m/s,0.rad/ s)

    val global = PinpointConfig

    var lastPose: Transform2D = Transform2D(0.0.m, 0.0.m, 0.0.m)

    init {
        device.initialize()
        device.recalibrateIMU()

        lastSDKPose = Pose2D(
            DistanceUnit.INCH, 0.0, 0.0,
            AngleUnit.DEGREES, 0.0
        )

        lastPose = Transform2D(0.0.m, 0.0.m, 0.0.m)

        device.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        device.setEncoderDirections(
            if (global.flipX) GoBildaPinpointDriver.EncoderDirection.REVERSED else GoBildaPinpointDriver.EncoderDirection.FORWARD,
            if (global.flipY) GoBildaPinpointDriver.EncoderDirection.REVERSED else GoBildaPinpointDriver.EncoderDirection.FORWARD
        )

        device.setOffsets(global.xOffset, global.yOffset)
    }

    fun getTransform(): Transform2D {
        return Transform2D(
            getSDKPose().getX(DistanceUnit.METER).m,
            getSDKPose().getY(DistanceUnit.METER).m,
            getSDKPose().getHeading(AngleUnit.RADIANS).rad
        )
    }

    val isDoneCalibrating: Boolean
        get() {
            return device.deviceStatus == GoBildaPinpointDriver.DeviceStatus.READY
        }

    private fun getSDKPose(): Pose2D {
        return lastSDKPose
    }

    fun update(pollPos: Boolean) {
        //TODO: REMEMBER TO DELETE THIS!!
//        device.setOffsets(global.xOffset, global.yOffset)

        if (pollPos) {
            device.update()
            velocity = Twist2D(
                device.velX.mm/s,
                device.velX.mm/s,
                device.headingVelocity.rad/s
            )
        } else {
            device.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING)
            lastPose
        }

        if (device.posX.isNaN()) {
            lastPose = Transform2D(
                lastSDKPose.getX(DistanceUnit.METER).m,
                lastSDKPose.getY(DistanceUnit.METER).m,
                lastSDKPose.getHeading(AngleUnit.RADIANS).rad
            )
        } else {
            val tempPose = device.position
            lastSDKPose = tempPose
        }
    }

    fun reset() {
        device.resetPosAndIMU()
        lastSDKPose = Pose2D(
            DistanceUnit.METER, 0.0, 0.0,
            AngleUnit.RADIANS, 0.0
        )
    }
}