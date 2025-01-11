package sigmacorns.common.io

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import eu.sirotin.kotunil.base.Second
import eu.sirotin.kotunil.derived.V
import net.unnamedrobotics.lib.hardware.impl.HardwareManagerImpl
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.Twist2D
import net.unnamedrobotics.lib.math2.tick
import net.unnamedrobotics.lib.rerun.RerunConnection
import sigmacorns.common.LoopTimes
import sigmacorns.common.subsystems.arm.ArmPose

class RobotIO(hardwareMap: HardwareMap, io: String = "127.0.0.1", private val initialArmPose: ArmPose? = null): SigmaIO() {
    val hardwareManager = HardwareManagerImpl(hardwareMap)
    val hubs: List<LynxModule>

    private val wheels = listOf("FL","FR","BR","BL")

    val drives = wheels.map { hardwareManager.motor("drive$it") }
    val turns = wheels.map { hardwareManager.crservo("servo$it")}
    val turnEncoders = wheels.map { hardwareMap.get(AnalogInput::class.java,"encoder$it") }
    val diffyServos = listOf(hardwareManager.servo( "diffyServoL"), hardwareManager.servo( "diffyServoR"))
    val armMotors = listOf(hardwareManager.motor("diffyMotorL"),hardwareManager.motor("diffyMotorR"))
    val armEncoders = armMotors.map { it.encoder }
    val clawServo = hardwareManager.servo("clawServo")

    override val drivePowers = drives.map { drive ->
        cachedActuator(LoopTimes.DRIVE_UPDATE_THRESHOLD) { drive.power = it }
    }
    override val turnPowers = turns.map { turn -> Actuator { power: Double -> turn.power = power } }
    override val armMotorPowers = armMotors.map { Actuator<Double> { power -> it.power = power } }
    override val diffyPos = diffyServos.map { Actuator<Double> { u -> it.position = u } }
    override fun time(): Second {
        TODO("Not yet implemented")
    }

    override val clawPos = Actuator<Double> { clawServo.position = it }

    private val armEncoderSensor = sensor(bulkReadable = true) { armEncoders.map { it.getCounts().tick } }
    context(ControlLoopContext<*,*,*,SigmaIO,*>) override fun armPositions() = armEncoderSensor.get()

    private val turnEncoderSensor = sensor(bulkReadable = true) { turnEncoders.map { it.voltage.V } }
    context(ControlLoopContext<*,*,*,SigmaIO,*>) override fun turnVoltages() = turnEncoderSensor.get()
    context(ControlLoopContext<*, *, *, SigmaIO,*>) override fun position(): Transform2D {
        TODO("Not yet implemented")
    }

    context(ControlLoopContext<*, *, *, SigmaIO,*>) override fun velocity(): Twist2D {
        TODO("Not yet implemented")
    }

    override fun clearBulkCache() = hubs.forEach { it.clearBulkCache() }

    init {
        turns.forEach {
            it.reverse(true)
        }

        drives.forEach {
            it.reverse(true)
        }

        armMotors[1].reverse(true)
        armEncoders[1].reverse(true)

        hubs = hardwareMap.getAll(LynxModule::class.java)

        hubs.forEach {
            it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
    }

    fun voltage() = hardwareManager.voltage().V

    override val rerunConnection = RerunConnection("lambda",io)
}