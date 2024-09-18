package sigmacorns.test

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.util.ElapsedTime
import net.hivemindrobotics.lib.control.controller.PIDController
import sigmacorns.common.control.swerve.angleToTick
import sigmacorns.common.control.swerve.tickToAngle
import kotlin.math.atan2
import kotlin.math.hypot

@TeleOp
class BasicSwerveTest: LinearOpMode() {
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

        waitForStart()

        val turnController = PIDController(PIDCoefficients(0.0001,0.0,0.0))
        val controllers = arrayOf(turnController.copy(),turnController.copy(),turnController.copy(),turnController.copy())
        val turns = arrayOf(turn1,turn2,turn3,turn4)
        val turnEncoders = arrayOf(turn1Encoder,turn2Encoder,turn3Encoder,turn4Encoder)

        //for + to be positive theta on the unit circle when looking top down encoders 1 and 2 need to be reversed.
        val reversedEncoders = arrayOf(-1,-1,1,1)
        val reversedPowers = arrayOf(-1,-1,-1,-1)

        turnEncoders.forEach {
            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        val timer = ElapsedTime()
        var angleTarget = false
        var wasLBumpPressed = false
        while (opModeIsActive()) {
            if(!wasLBumpPressed && gamepad1.left_bumper) angleTarget = !angleTarget
            wasLBumpPressed = gamepad1.left_bumper

            if(gamepad1.a) drive1.power = 1.0
            if(gamepad1.b) drive2.power = 1.0
            if(gamepad1.x) drive3.power = 1.0
            if(gamepad1.y) drive4.power = 1.0

            val pow = if(angleTarget)
                hypot(gamepad1.left_stick_x.toDouble(),gamepad1.left_stick_y.toDouble())
            else gamepad1.left_stick_y.toDouble()
            drive1.power = pow
            drive2.power = -pow
            drive3.power = pow
            drive4.power = pow

            if(gamepad1.right_bumper) {
                controllers.forEach { it.target = 0.0; it.position=0.0; }
                turnEncoders.forEach {
                    it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                    it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                }
            }

            if(gamepad1.dpad_up) turn1.power = 1.0
            else if(gamepad1.dpad_down) turn2.power = 1.0
            else if(gamepad1.dpad_left) turn3.power = 1.0
            else if(gamepad1.dpad_right) turn4.power = 1.0
            else for (i in controllers.indices) {
                val pos = reversedEncoders[i]*turnEncoders[i].currentPosition.toDouble()
                val theta = atan2(gamepad1.left_stick_y.toDouble(),gamepad1.left_stick_x.toDouble())
                    .takeUnless { it.isNaN() }
                    ?: 0.0
                if(angleTarget) controllers[i].target = angleToTick(pos,theta) else controllers[i].target -= gamepad1.left_stick_x*400.0
                controllers[i].position = pos
                turns[i].power = reversedPowers[i]*controllers[i].update(timer.seconds())
            }
            for(i in controllers.indices) {
                dashTelemetry.addData("Wheel " + (i+1),
                    "target = " + controllers[i].target +
                           " pos = " + controllers[i].position +
                           " power = " + turns[i].power +
                           "\n angle = " + tickToAngle(reversedEncoders[i]*turnEncoders[i].currentPosition)
                )
            }

            dashTelemetry.update()

            timer.reset()
        }
    }
}
