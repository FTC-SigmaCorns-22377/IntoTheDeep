package sigmacorns.opmode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.ValueProvider
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import kotlinx.coroutines.yield
import java.util.function.Consumer
import java.util.function.DoubleConsumer
import java.util.function.DoubleSupplier


@TeleOp
class CRAZY: LinearOpMode() {
    override fun runOpMode() {
        hardwareMap.dcMotor.entrySet()
            .forEach(Consumer<Map.Entry<String?, DcMotor>> { entry: Map.Entry<String?, DcMotor> ->
                val motor = entry.value
                FtcDashboard.getInstance().addConfigVariable(
                    " Motors",
                    entry.key,
                    Provider({ motor.power }, { power: Double -> motor.power = power })
                )
            })
        hardwareMap.servo.entrySet()
            .forEach(Consumer<Map.Entry<String?, Servo>> { entry: Map.Entry<String?, Servo> ->
                val servo: Servo = entry.value
                FtcDashboard.getInstance().addConfigVariable(
                    " Servos",
                    entry.key,
                    Provider(servo::getPosition, servo::setPosition)
                )
            })
        hardwareMap.crservo.entrySet()
            .forEach(Consumer<Map.Entry<String?, CRServo>> { entry: Map.Entry<String?, CRServo> ->
                val CRServo: CRServo = entry.value
                FtcDashboard.getInstance().addConfigVariable(
                    " CRServos",
                    entry.key,
                    Provider(CRServo::getPower, CRServo::setPower)
                )
            })

        waitForStart()

        while(opModeIsActive()) {

        }
    }
    private class Provider constructor(
        private val get: DoubleSupplier,
        private val set: DoubleConsumer
    ) :
        ValueProvider<Double?> {
        override fun get(): Double {
            return get.asDouble
        }

        override fun set(value: Double?) {
            set.accept(value!!)
        }
    }

}