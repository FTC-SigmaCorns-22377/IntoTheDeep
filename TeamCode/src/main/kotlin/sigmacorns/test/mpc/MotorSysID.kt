package sigmacorns.test.mpc

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.lynx.LynxDcMotorController
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.configuration.LynxConstants
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import java.lang.reflect.InvocationTargetException
import java.lang.reflect.Method
import java.util.Collections
import kotlin.math.sign


@TeleOp
@Config
class MotorSysID : LinearOpMode() {
    var dashboard: FtcDashboard = FtcDashboard.getInstance()
    var dashboardTelemetry: Telemetry = dashboard.getTelemetry()

    companion object Config {
        @JvmStatic
        var t_fric: Double = 0.00565816;
        @JvmStatic
        val TICKS_PER_REV: Double = 377.7
        @JvmStatic
        var k_t: Double = 0.0188605
        @JvmStatic
        var R: Double = 1.15674
        @JvmStatic
        var J: Double = 0.0025
        @JvmStatic
        var L: Double = 0.00005
        @JvmStatic
        var B: Double = 0.0
        @JvmStatic
        var t_c: Double = 40.0

        @JvmStatic
        var g_r: Double = 1.0

        @JvmStatic
        var LOW_PASS: Double = 0.5
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val motor = hardwareMap["driveBackRight"] as DcMotorEx

        val timer = ElapsedTime()
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        var velLowPass = 0.0
        var oldPos = 0.0
        waitForStart()

        while (opModeIsActive()) {
            val power = -gamepad1.left_stick_y.toDouble()
            if (gamepad1.a) {
                velEstimate = 0.0
                IEstimate = 0.0
            }

            val dt = timer.seconds()
            timer.reset()
            motor.power = power

            //fix direction later
            val velocity = -getMotorlVel(motor, 3, TICKS_PER_REV)
            val pos = motor.currentPosition.toDouble()
            val vel_from_ticks = ((pos - oldPos) * Math.PI * 2.0) / (TICKS_PER_REV * dt)
            oldPos = pos
            velLowPass = velLowPass * LOW_PASS + vel_from_ticks * (1 - LOW_PASS)
            dashboardTelemetry.addData("velocity (rad/s)", velocity)
            dashboardTelemetry.addData("velocity low pass (rad/s)", velLowPass)

            val voltage = voltage()
            val current = motor.getCurrent(CurrentUnit.AMPS)
            dashboardTelemetry.addData("voltage", voltage)
            dashboardTelemetry.addData("power (V)", power * voltage)
            dashboardTelemetry.addData("motorCurrent", current)
            dashboardTelemetry.addData("loop time (ms)", dt * 1000.0)

            // velEstimate = velocity;
            if (gamepad1.b) {
                velEstimate = velocity
                IEstimate = current
                //                power = 0;
            }
            runModel(voltage * power, dt)

            dashboardTelemetry.addData("vel estimate (nm)", velEstimate)
            dashboardTelemetry.addData("I estimate (amps)", IEstimate)


            dashboardTelemetry.update()
        }
    }

    //because for some reason the conversion from ticks to velocity dosent work properly on this motor
    //maybe the thing about not every motor supporting DcMotorControllerEx?
    //(probably?) should cache reflection stuff, doing it everytime is inefficinet
    fun getMotorlVel(motor: DcMotorEx, port: Int, ticksPerRev: Double): Double {
        val controller = motor.controller as LynxDcMotorController
        try {
            //one of the few times runtime reflection is useful
            val method: Method = LynxDcMotorController::class.java.getDeclaredMethod(
                "internalGetMotorTicksPerSecond",
                Int::class.javaPrimitiveType
            )
            method.setAccessible(true)
            //idk if this double casting is needed, putting it for safety
            val x = (method.invoke(
                controller,
                port - LynxConstants.INITIAL_MOTOR_PORT
            ) as Int).toDouble()
            //output in rads
            return (x / ticksPerRev) * Math.PI * 2.0
        } catch (e: NoSuchMethodException) {
            e.printStackTrace()
        } catch (e: IllegalAccessException) {
            e.printStackTrace()
        } catch (e: InvocationTargetException) {
            e.printStackTrace()
        }
        //rethink if fail condition should cause error or not
        return 0.0
    }


    var MODEL_STEP_COUNT: Int = 1000
    fun voltage(): Double {
        val result = 0.0
        val voltages = ArrayList<Double>()
        for (voltageSensor in hardwareMap.voltageSensor) {
            voltages.add(voltageSensor.voltage)
        }
        Collections.sort(voltages)
        return voltages[voltages.size / 2]
    }

    var velEstimate: Double = 0.0
    var IEstimate: Double = 0.0

    //TODO: actually solve the differential equation instead of approximating it bc its unstable
    fun runModel(volt: Double, dt: Double) {
        for (i in 0 until MODEL_STEP_COUNT) {
            val emf = -k_t * (velEstimate * g_r) / L
            dashboardTelemetry.addData("emf,", emf * L)

            val dI = emf - (IEstimate * R) / L + volt / L
            dashboardTelemetry.addData("without emf", (dI - emf) * L)
            //B and t_c are proportional to J, but that is neglected for tuning purposes bc it is easier to find t_c and B first.
            val dV = -B * velEstimate - sign(velEstimate) * t_c + (IEstimate * k_t * g_r) / J

            IEstimate += dI * (dt / MODEL_STEP_COUNT.toDouble())

            //            IEstimate = Math.max(IEstimate,0);
            velEstimate += dV * (dt / MODEL_STEP_COUNT.toDouble())
        }
    }
}