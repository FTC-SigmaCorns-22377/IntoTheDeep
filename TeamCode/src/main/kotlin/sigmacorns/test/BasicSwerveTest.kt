package sigmacorns.test

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.util.ElapsedTime
import net.unnamedrobotics.lib.control.controller.PIDController
import net.unnamedrobotics.lib.math.Vector2
import net.unnamedrobotics.lib.math.radians
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians
import sigmacorns.common.swerve.TICKS_PER_REV
import sigmacorns.common.swerve.tickToAngle
import kotlin.math.PI
import kotlin.math.absoluteValue

@TeleOp
class BasicSwerveTest: LinearOpMode() {
    override fun runOpMode() {
        val drive1 = hardwareMap.get(DcMotor::class.java, "m1")
        val drive2 = hardwareMap.get(DcMotor::class.java, "m2")
        val drive3 = hardwareMap.get(DcMotor::class.java, "m3")
        val drive4 = hardwareMap.get(DcMotor::class.java, "m4")

        val octo = hardwareMap.get(OctoQuad::class.java, "octo")

        val turn1Encoder = hardwareMap.get(DcMotor::class.java, "m8")
        val turn2Encoder = hardwareMap.get(DcMotor::class.java, "m4")
        val turn3Encoder = hardwareMap.get(DcMotor::class.java, "m1")
        val turn4Encoder = hardwareMap.get(DcMotor::class.java, "m5")

        val turn1 = hardwareMap.get(CRServo::class.java, "t1")
        val turn2 = hardwareMap.get(CRServo::class.java, "t3")
        val turn3 = hardwareMap.get(CRServo::class.java, "t2")
        val turn4 = hardwareMap.get(CRServo::class.java, "t4")

        val turnDirs = arrayOf(
            Vector2.fromAngle(7* PI /4.0,1.0),
            Vector2.fromAngle(5* PI /4.0,1.0),
            Vector2.fromAngle(3* PI /4.0,1.0),
            Vector2.fromAngle(1* PI /4.0,1.0),
        )

        val dashboard = FtcDashboard.getInstance()
        val dashTelemetry = dashboard.telemetry

        val imu = hardwareMap.get(IMU::class.java,"imu")
        val params = IMU.Parameters(
            RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ))
        imu.initialize(params)

        waitForStart()

        val turnController = PIDController(PIDCoefficients(0.0001,0.0,0.0))
        val controllers: Array<PIDController> = arrayOf(turnController.copy() as PIDController,turnController.copy() as PIDController,turnController.copy() as PIDController,turnController.copy() as PIDController)
        val turns = arrayOf(turn1,turn2,turn3,turn4)
        val turnEncoders = arrayOf(turn1Encoder,turn2Encoder,turn3Encoder,turn4Encoder)
        val drives = arrayOf(drive1,drive2,drive3,drive4)

        //for + to be positive theta on the unit circle when looking top down encoders 1 and 2 need to be reversed.
        val reversedEncoders = arrayOf(-1,-1,1,1)
        val reversedPowers = arrayOf(-1,-1,-1,-1)
        val reversedDrives = arrayOf(1.0,-1.0,1.0,1.0)

        turnEncoders.forEach {
            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        val timer = ElapsedTime()
        var angleTarget = false
        var wasLBumpPressed = false
        var vs = listOf(Vector2(),Vector2(),Vector2(),Vector2())
        while (opModeIsActive()) {
            if(!wasLBumpPressed && gamepad1.left_bumper) angleTarget = !angleTarget
            wasLBumpPressed = gamepad1.left_bumper

            if(gamepad1.back) imu.resetYaw()
            val heading = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)

            if(gamepad1.right_bumper) {
                controllers.forEach { it.target = 0.0; it.position=0.0; }
                turnEncoders.forEach {
                    it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                    it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                }
            }

            if(gamepad1.a) drive1.power = 1.0
            else if(gamepad1.b) drive2.power = 1.0
            else if(gamepad1.x) drive3.power = 1.0
            else if(gamepad1.y) drive4.power = 1.0
            else if(gamepad1.dpad_up) turn1.power = 1.0
            else if(gamepad1.dpad_down) turn2.power = 1.0
            else if(gamepad1.dpad_left) turn3.power = 1.0
            else if(gamepad1.dpad_right) turn4.power = 1.0
            else {
                val transform = Vector2(-gamepad1.left_stick_x,gamepad1.left_stick_y).rotate(-heading.radians)
                val turn = gamepad1.right_trigger-gamepad1.left_trigger
                val keepOrientation = transform.magnitude + turn.absoluteValue < 0.001
                val locked = gamepad1.left_stick_button
                if (!keepOrientation) vs = turnDirs.map { it*-turn.toDouble() + transform }
                val maxMag = vs.maxBy { it.magnitude }.magnitude
                if(maxMag>1.0) vs = vs.map { it/maxMag }

                for (i in controllers.indices) {
                    controllers[i].coefficients = PIDCoefficients(0.0001,0.0,0.0)

                    val pos = reversedEncoders[i]*turnEncoders[i].currentPosition.toDouble()
                    if(angleTarget) {
                        var target = if(locked) turnDirs[i].angleFromOrigin+PI/2.0 else vs[i].angleFromOrigin
                        val cur = tickToAngle(pos.toInt())
                        var diff = normalizeRadians(target - cur)
                        val flipped = diff.absoluteValue > PI/2.0
                        if(flipped) target = normalizeRadians(target - PI)
                        diff = normalizeRadians(target - cur)
                        var driveDir = (if(flipped) -1 else 1)*reversedDrives[i]
                        if (keepOrientation || locked) driveDir = 0.0
                        val targetTicks = pos + diff/(2*PI) * TICKS_PER_REV
                        controllers[i].target = targetTicks
                        drives[i].power = vs[i].magnitude*driveDir
                    } else {
                        controllers[i].target -= gamepad1.left_stick_x*400.0
                        val pow = gamepad1.left_stick_y.toDouble()
                        drives[i].power = pow*reversedDrives[i]
                    }

                    controllers[i].position = pos
                    turns[i].power = reversedPowers[i]*controllers[i].update(timer.seconds())
                }
            }

            for(i in controllers.indices) {
                dashTelemetry.addData("Wheel " + (i+1),
                    "target = " + controllers[i].target +
                           " pos = " + controllers[i].position +
                           " power = " + turns[i].power +
                           "\n angle = " + tickToAngle(reversedEncoders[i]*turnEncoders[i].currentPosition)
                )

                dashTelemetry.addData("Wheel " + (i+1) + " error: ",controllers[i].target-controllers[i].position)
            }

            dashTelemetry.update()

            timer.reset()
        }
    }
}