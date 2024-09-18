package sigmacorns.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.*

@TeleOp
class Swerve: LinearOpMode() {

    class Point(var x: Double = 0.0, var y: Double = 0.0);

    class SwerveConfig {
        var NUM_MODULES: Int = 4;
        var module_offsets: Array<Point> = Array(NUM_MODULES) { Point() };
    }

    data class ModuleState(var velocity: Double = 0.0, var theta: Double = 0.0, var omega: Double = 0.0);

    fun moduleStates(config: SwerveConfig, vx: Double, vy: Double, omega: Double): Array<ModuleState> {
        // https://www.overleaf.com/read/hjncvyqkrvzn#01f1cd
        var velocities: Array<ModuleState> = Array(config.NUM_MODULES) { ModuleState() };

        for (i in 0..<config.NUM_MODULES) {
            var vx_i = vx + omega * (-config.module_offsets[i].y);
            var vy_i = vy + omega * config.module_offsets[i].x;
            velocities[i] = ModuleState(sqrt(vx_i * vx_i + vy_i * vy_i), atan2(vy_i, vx_i));
        }

        return velocities;
    }

    override fun runOpMode() {
        val N = 4;
        val MAX_SPEED = 2.0; // m/s
        val MAX_ANGULAR_SPEED = PI; // rad/s
        var TICKS_PER_REVOLUTION = 2048.0;
        var MOTOR_TURNS_PER_WHEEL_TURN = 13.0; // 13 turns of drive motor and encoder --> 1 turn of drive wheel
        var ENCODER_TURNS_PER_AZIMUTH_TURN = 2.5; // 2.5 turns of encoder --> 1 turn of azimuth
        var WHEEL_RADIUS = 0.03; // estimate

        val driveMotors = arrayOf(hardwareMap.get(DcMotor::class.java, "m1"),
                                  hardwareMap.get(DcMotor::class.java, "m2"),
                                  hardwareMap.get(DcMotor::class.java, "m3"),
                                  hardwareMap.get(DcMotor::class.java, "m4"));

        val driveEncoders = arrayOf(hardwareMap.get(DcMotorEx::class.java, "m8"),
                                    hardwareMap.get(DcMotorEx::class.java, "m5"),
                                    hardwareMap.get(DcMotorEx::class.java, "m1"),
                                    hardwareMap.get(DcMotorEx::class.java, "m4"));// TODO fix indices

        val turnMotors = arrayOf(hardwareMap.get(CRServo::class.java, "t1"),
                                 hardwareMap.get(CRServo::class.java, "t3"),
                                 hardwareMap.get(CRServo::class.java, "t2"),
                                 hardwareMap.get(CRServo::class.java, "t4"));

        val turnEncoders = arrayOf(hardwareMap.get(DcMotor::class.java, "m1"),
                                   hardwareMap.get(DcMotor::class.java, "m8"),
                                   hardwareMap.get(DcMotor::class.java, "m5"),
                                   hardwareMap.get(DcMotor::class.java, "m4"));

        val inverts = arrayOf(1,-1,1,-1)

        // TODO reset encoders

        val config = SwerveConfig();
        config.module_offsets = arrayOf(Point(-1.0, -1.0), Point(-1.0, 1.0), Point(1.0, -1.0), Point(1.0, 1.0)); // these offsets are obviously fake but whatever
        // units should match whatever units we want for velocity

        waitForStart()

        val turnController = PIDController(PIDFCoefficients(0.01, 0.0, 0.0, 0.0)) // TODO tune constants
        val driveController = PIDController(PIDFCoefficients(0.01, 0.0, 0.0, 0.05)) // TODO tune constants

        var turnControllers = arrayOf(turnController.copy(), turnController.copy(), turnController.copy(), turnController.copy())
        var driveControllers = arrayOf(driveController.copy(), driveController.copy(), driveController.copy(), driveController.copy())
        val timer = ElapsedTime()

        while (opModeIsActive()) {

            val magnitude = sqrt(gamepad1.left_stick_x.toDouble() * gamepad1.left_stick_x.toDouble() + gamepad1.left_stick_y.toDouble() * gamepad1.left_stick_y.toDouble()) * (MAX_SPEED / sqrt(2.0)); // max of sqrt(2) from joystick I believe
            var angle = atan2(gamepad1.left_stick_y.toDouble(), gamepad1.left_stick_x.toDouble());

            // convert to actual speeds from joystick values
            var vx = cos(angle) * magnitude;
            var vy = sin(angle) * magnitude;
            var omega = gamepad1.right_stick_x.toDouble() * MAX_ANGULAR_SPEED;

            val states = moduleStates(config, vx, vy, omega);

            for (i in 0..<config.NUM_MODULES) {
                // Velocity PID on drive motor
                driveControllers[i].target = states[i].velocity;
                driveControllers[i].position = driveEncoders[i].velocity.toDouble() * (1 / TICKS_PER_REVOLUTION) * (1 / MOTOR_TURNS_PER_WHEEL_TURN) * (2.0 * PI * WHEEL_RADIUS); // Convert from ticks per second to meters per second (ticks/second * revolutions at encoder/tick * revolutions at wheel/revolutions at encoder * meters/revolution at wheel)
                driveMotors[i].power = driveControllers[i].update(timer.seconds());

                // Position PID on turn motor
                turnControllers[i].target = states[i].theta;
                turnControllers[i].position = turnEncoders[i].currentPosition.toDouble() * (1 / TICKS_PER_REVOLUTION) * (1 / ENCODER_TURNS_PER_AZIMUTH_TURN); // Convert from ticks to theta (ticks * encoder turns/tick * azimuth turns/encoder turn)
                turnMotors[i].power = turnControllers[i].update(timer.seconds());
            }

            timer.reset()

        }
    }
}