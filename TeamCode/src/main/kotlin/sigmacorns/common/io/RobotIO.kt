package sigmacorns.common.io

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import eu.sirotin.kotunil.base.Second
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.V
import net.unnamedrobotics.lib.driver.gobilda.GoBildaPinpointDriver
import net.unnamedrobotics.lib.hardware.impl.HardwareManagerImpl
import net.unnamedrobotics.lib.hardware.interfaces.Servo
import net.unnamedrobotics.lib.math2.Twist2D
import net.unnamedrobotics.lib.math2.tick
import net.unnamedrobotics.lib.rerun.RerunConnection
import net.unnamedrobotics.lib.rerun.rerun
import net.unnamedrobotics.lib.util.Clock
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import sigmacorns.common.LoopTimes
import sigmacorns.common.subsystems.arm.DiffyOutputPose
import sigmacorns.common.subsystems.arm.ScoringPose
import sigmacorns.common.subsystems.arm.boxTubeKinematics

class RobotIO(
    val hardwareMap: HardwareMap,
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
    val diffyServos: List<Servo> = listOf(hardwareManager.servo( "diffyServoL"), hardwareManager.servo( "diffyServoR"))
    val diffyServosReversed = listOf(true,false)
    val armMotors = listOf(hardwareManager.motor("diffyMotorL"),hardwareManager.motor("diffyMotorR"))
    val armEncoders = armMotors.map { it.encoder }
    val clawServo: Servo? = hardwareManager.servo("clawServo")
    val pinpointDriver = hardwareMap.get(
        GoBildaPinpointDriver::class.java,"pinpoint")
    val pinpoint = PinpointLocalizer(pinpointDriver)

    override val drivePowers = drives.map { drive ->
        cachedActuator(LoopTimes.DRIVE_UPDATE_THRESHOLD) { drive.power = it }
    }

    override val turnPowers = turns.map { turn -> cachedActuator(LoopTimes.TURN_UPDATE_THRESHOLD) { power: Double -> turn.power = power } }
    override val armMotorPowers = armMotors.map { cachedActuator(LoopTimes.ARM_UPDATE_THRESHOLD) { power -> it.power = power } }
    override val diffyPos =
        diffyServos
            .zip(diffyServosReversed)
            .map { cachedActuator(LoopTimes.DIFFY_UPDATE_THRESHOLD) { u ->
                it.first.position = (if(it.second) 1-u else u)
            } }

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

    private val posSensor = sensor {
        pinpoint.update(true)
        pinpoint.getTransform()
    }
    context(ControlLoopContext<*, *, *, SigmaIO,*>) override fun position() = posSensor.get()

    private val velSensor = sensor { pinpoint.velocity }
    context(ControlLoopContext<*, *, *, SigmaIO,*>) override fun velocity(): Twist2D = velSensor.get()

    override fun clearBulkCache() = hubs.forEach { it.clearBulkCache() }

    override val rerunConnection = RerunConnection(rerunName,io)

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

        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        if(initialScoringPose!=null) pinpointDriver.setPosition(initialScoringPose.robotPos.let {
            Pose2D(
                DistanceUnit.METER,
                it.x.value,it.y.value,
                AngleUnit.RADIANS,
                initialScoringPose.theta.value
            )
        })
        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,GoBildaPinpointDriver.EncoderDirection.REVERSED)

        rerun(rerunConnection) {
            field(hardwareMap.appContext)
        }

        hubs = hardwareMap.getAll(LynxModule::class.java)

        hubs.forEach {
            it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
    }

    override fun voltage() = 12.V

}