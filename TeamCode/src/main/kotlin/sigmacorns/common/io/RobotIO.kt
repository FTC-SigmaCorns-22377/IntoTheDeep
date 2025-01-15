package sigmacorns.common.io

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import eu.sirotin.kotunil.base.Second
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.V
import net.unnamedrobotics.lib.hardware.impl.HardwareManagerImpl
import net.unnamedrobotics.lib.hardware.interfaces.Servo
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.Twist2D
import net.unnamedrobotics.lib.math2.tick
import net.unnamedrobotics.lib.rerun.RerunConnection
import net.unnamedrobotics.lib.util.Clock
import sigmacorns.common.LoopTimes
import sigmacorns.common.subsystems.arm.DiffyOutputPose
import sigmacorns.common.subsystems.arm.ScoringPose
import sigmacorns.common.subsystems.arm.boxTubeKinematics

class RobotIO(
    hardwareMap: HardwareMap,
    io: String = "192.168.43.122",
    rerunName: String = "unnamed",
    private val initialScoringPose: ScoringPose? = null
): SigmaIO() {
    val hardwareManager = HardwareManagerImpl(hardwareMap)
    val hubs: List<LynxModule>

    private val wheels = listOf("FL","FR","BL","BR")

    val drives = wheels.map { hardwareManager.motor("drive$it") }
    val turns = wheels.map { hardwareManager.crservo("servo$it")}
    val turnEncoders = wheels.map { hardwareMap.get(AnalogInput::class.java,"encoder$it") }
    val diffyServos: List<Servo>? = null // listOf(hardwareManager.servo( "diffyServoL"), hardwareManager.servo( "diffyServoR"))
    val armMotors = listOf(hardwareManager.motor("diffyMotorL"),hardwareManager.motor("diffyMotorR"))
    val armEncoders = armMotors.map { it.encoder }
    val clawServo: Servo? = null //hardwareManager.servo("clawServo")

    override val drivePowers = drives.map { drive ->
        cachedActuator(LoopTimes.DRIVE_UPDATE_THRESHOLD) { drive.power = it }
    }

    override val turnPowers = turns.map { turn -> cachedActuator(LoopTimes.TURN_UPDATE_THRESHOLD) { power: Double -> turn.power = power } }
    override val armMotorPowers = armMotors.map { cachedActuator(LoopTimes.ARM_UPDATE_THRESHOLD) { power -> it.power = power } }
    override val diffyPos =
        diffyServos?.map { cachedActuator(LoopTimes.DIFFY_UPDATE_THRESHOLD) { u -> it.position = u } }
            ?: List(4) { Actuator<Double> { } }

    override fun time(): Second = Clock.seconds.s

    override val clawPos = Actuator<Double> { clawServo?.position = it }

    var armEncoderOffsets = listOf(0,0)
    private val armEncoderSensor = sensor(bulkReadable = true) {
        armEncoders
            .zip(armEncoderOffsets)
            .map { (it.first.getCounts() + it.second).tick }
    }
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

        if(initialScoringPose!=null) {
            val cur = boxTubeKinematics.inverse(
                DiffyOutputPose(initialScoringPose.pivot,initialScoringPose.extension)
            )

            val offset1 = -armEncoders[0].getCounts().tick + cur.axis1
            val offset2 = -armEncoders[1].getCounts().tick + cur.axis2

            armEncoderOffsets = listOf(
                offset1.value.toInt(),
                offset2.value.toInt()
            )
        }

        hubs = hardwareMap.getAll(LynxModule::class.java)

        hubs.forEach {
            it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
    }

    override fun voltage() = 12.V

    override val rerunConnection = RerunConnection(rerunName,io)
}