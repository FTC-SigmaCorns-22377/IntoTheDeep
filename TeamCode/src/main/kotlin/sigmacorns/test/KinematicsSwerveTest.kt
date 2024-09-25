package sigmacorns.test

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.util.ElapsedTime
import net.unnamedrobotics.lib.math.Vector2
import net.unnamedrobotics.lib.math.radians
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import sigmacorns.common.control.swerve.Swerve
@TeleOp
class KinematicsSwerveTest: LinearOpMode(){
    override fun runOpMode() {
        val drive1 = hardwareMap.get(DcMotor::class.java, "m1")
        val drive2 = hardwareMap.get(DcMotor::class.java, "m2")
        val drive3 = hardwareMap.get(DcMotor::class.java, "m3")
        val drive4 = hardwareMap.get(DcMotor::class.java, "m4")

        val turn1Encoder = hardwareMap.get(DcMotor::class.java, "m8")
        val turn2Encoder = hardwareMap.get(DcMotor::class.java, "m4")
        val turn3Encoder = hardwareMap.get(DcMotor::class.java, "m1")
        val turn4Encoder = hardwareMap.get(DcMotor::class.java, "m5")

        val turn1 = hardwareMap.get(CRServo::class.java, "t1")
        val turn2 = hardwareMap.get(CRServo::class.java, "t3")
        val turn3 = hardwareMap.get(CRServo::class.java, "t2")
        val turn4 = hardwareMap.get(CRServo::class.java, "t4")

        val dashboard = FtcDashboard.getInstance()
        val dashTelemetry = dashboard.telemetry

        val controller = Swerve(
            arrayOf(drive1,drive2,drive3,drive4),
            arrayOf(turn1,turn2,turn3,turn4),
            arrayOf(turn1Encoder,turn2Encoder,turn3Encoder,turn4Encoder)
        )

        val timer = ElapsedTime()

        val imu = hardwareMap.get(IMU::class.java,"imu")
        val params = IMU.Parameters(
            RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            )
        )
        imu.initialize(params)

        waitForStart()

        while (opModeIsActive()) {
            if(gamepad1.right_bumper) controller.reset()
            val heading = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)

            controller.update(
                Vector2(gamepad1.left_stick_x,gamepad1.left_stick_y).rotate(-heading.radians),
                (gamepad1.right_trigger-gamepad1.left_trigger).toDouble(),
                gamepad1.left_stick_button,
                timer.seconds()
            )
            controller.telemetry(dashTelemetry)
            dashTelemetry.update()

            timer.reset()
        }
    }
}