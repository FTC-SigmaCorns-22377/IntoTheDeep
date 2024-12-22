package sigmacorns.common.io

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import eu.sirotin.kotunil.derived.V
import net.unnamedrobotics.lib.hardware.impl.HardwareManagerImpl
import net.unnamedrobotics.lib.math2.tick
import net.unnamedrobotics.lib.rerun.RerunConnection
import net.unnamedrobotics.lib.util.Clock
import sigmacorns.common.RobotTickI
import sigmacorns.common.RobotTickO

class RobotIO(hardwareMap: HardwareMap): SigmaIO {
    private val hardwareManager = HardwareManagerImpl(hardwareMap)

    val drive1 = hardwareManager.motor("drive1")
    val drive2 = hardwareManager.motor("drive2")
    val drive3 = hardwareManager.motor("drive3")
    val drive4 = hardwareManager.motor("drive4")
    val drives = listOf(drive1,drive2,drive3,drive4)

    val turn1 = hardwareManager.crservo("turn1")
    val turn2 = hardwareManager.crservo("turn2")
    val turn3 = hardwareManager.crservo("turn3")
    val turn4 = hardwareManager.crservo("turn4")
    val turns = listOf(turn1,turn2,turn3,turn4)

    val turn1Encoder = hardwareMap.get(AnalogInput::class.java,"te1")
    val turn2Encoder = hardwareMap.get(AnalogInput::class.java,"te2")
    val turn3Encoder = hardwareMap.get(AnalogInput::class.java,"te3")
    val turn4Encoder = hardwareMap.get(AnalogInput::class.java,"te4")
    val turnEncoders = listOf(turn1Encoder,turn2Encoder,turn3Encoder,turn4Encoder)

//    val diffyLeft = hardwareManager.servo( "dl")
//    val diffyRight = hardwareManager.servo("dr")
//    val diffys = listOf(diffyLeft, diffyRight)

    val armMotor1 = hardwareManager.motor("arm1")
    val armMotor2 = hardwareManager.motor("arm2")

    fun voltage() = hardwareManager.voltage().V

    override val rerunConnection = RerunConnection("lambda","127.0.0.1")

    override fun update(o: RobotTickO): RobotTickI {
        o.drivePowers.zip(drives).forEach { it.second.power = it.first }
        o.turnPowers.zip(turns).forEach { it.second.power = it.first }
        o.armPowers.zip(listOf(armMotor1,armMotor2)).forEach { it.second.power = it.first }

        return RobotTickI(
            o.nextState,
            t = Clock.milliseconds,
            v = voltage(),
            turnEncodersPos = turnEncoders.map { it.voltage.V },
            armMotor1Pos = armMotor1.pos().tick,
            armMotor2Pos = armMotor2.pos().tick
        )
    }

//    val pinpoint = hardwareManager.pinpoint("pin")

//    init {
//        val params = IMU.Parameters(
//            RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
//            )
//        )
//        imu.initialize(params)
//    }

}