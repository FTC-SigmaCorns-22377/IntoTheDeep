package sigmacorns.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.*
import kotlinx.coroutines.runBlocking
import net.unnamedrobotics.lib.command.Scheduler
import net.unnamedrobotics.lib.math.radians
import net.unnamedrobotics.lib.math2.cast
import sigmacorns.common.Robot
import sigmacorns.common.io.RobotIO
import sigmacorns.common.io.SigmaIO
import sigmacorns.opmode.SimOrHardwareOpMode

@TeleOp(name = "_APRILTAG_")
class AprilTagTest: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {
        val robot = Robot(io)
        val lime = (io as RobotIO).limelight
        val imu = io.imu
        telemetry.setMsTransmissionInterval(10)
        lime.start()

        Scheduler.reset()

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addData("Corners", "Corners %f", lime.latestResult.fiducialResults.first().targetCorners)
            telemetry.addData("Heading", "Corners %f", imu.robotYawPitchRollAngles.yaw.radians)}
    }}