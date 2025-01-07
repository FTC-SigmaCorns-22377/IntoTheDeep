package sigmacorns.common.io

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import eu.sirotin.kotunil.derived.V
import net.unnamedrobotics.lib.hardware.impl.HardwareManagerImpl
import net.unnamedrobotics.lib.math2.tick
import net.unnamedrobotics.lib.rerun.RerunConnection
import net.unnamedrobotics.lib.util.Clock
import sigmacorns.common.ControllerState
import sigmacorns.common.RobotTickI
import sigmacorns.common.RobotTickO
import sigmacorns.common.subsystems.arm.ArmPose
import sigmacorns.common.subsystems.arm.DiffyOutputPose

class RobotIO(hardwareMap: HardwareMap, io: String = "127.0.0.1", private val initialArmPose: ArmPose? = null): SigmaIO {
    private val hardwareManager = HardwareManagerImpl(hardwareMap)

    val drive1 = hardwareManager.motor("driveFL")
    val drive2 = hardwareManager.motor("driveFR")
    val drive3 = hardwareManager.motor("driveBR")
    val drive4 = hardwareManager.motor("driveBL")
    val drives = listOf(drive1,drive2,drive3,drive4)

    val turn1 = hardwareManager.crservo("servoFL")
    val turn2 = hardwareManager.crservo("servoFR")
    val turn3 = hardwareManager.crservo("servoBR")
    val turn4 = hardwareManager.crservo("servoBL")
    val turns = listOf(turn1,turn2,turn3,turn4)

    val turn1Encoder = hardwareMap.get(AnalogInput::class.java,"encoderFL")
    val turn2Encoder = hardwareMap.get(AnalogInput::class.java,"encoderFR")
    val turn3Encoder = hardwareMap.get(AnalogInput::class.java,"encoderBR")
    val turn4Encoder = hardwareMap.get(AnalogInput::class.java,"encoderBL")
    val turnEncoders = listOf(turn1Encoder,turn2Encoder,turn3Encoder,turn4Encoder)


    val diffyLeft = hardwareManager.servo( "diffyServoL")
    val diffyRight = hardwareManager.servo("diffyServoR")
    val diffys = listOf(diffyLeft, diffyRight)

    val armMotor1 = hardwareManager.motor("diffyMotorL")
    val armMotor2 = hardwareManager.motor("diffyMotorR")

    val clawServo = hardwareManager.servo("clawServo")

    val armEncoder1 = armMotor1.encoder
    val armEncoder2 = armMotor2.encoder

    init {
        turns.forEach {
            it.reverse(true)
        }

        drives.forEach {
            it.reverse(true)
        }

        armMotor2.reverse(true)
        armEncoder2.reverse(true)

    }

    fun voltage() = hardwareManager.voltage().V

    override val rerunConnection = RerunConnection("lambda",io)

    override fun update(o: RobotTickO): RobotTickI {
        o.drivePowers.zip(drives).forEach { it.second.power = it.first }
        o.turnPowers.zip(turns).forEach { it.second.power = it.first }
        o.armPowers.zip(listOf(armMotor1,armMotor2)).forEach { it.second.power = it.first }
        diffyLeft.position = o.diffyClawPos.lAngle
        diffyRight.position = o.diffyClawPos.rAngle
        clawServo.position = o.claw


        return RobotTickI(
            o.nextState,
            t = Clock.milliseconds,
            v = voltage(),
            turnEncodersPos = turnEncoders.map { it.voltage.V },
            armMotor1Pos = armEncoder1.getCounts().tick,
            armMotor2Pos = armEncoder2.getCounts().tick
        )
    }

    fun initial(): RobotTickI {
        val controllerState = ControllerState()
        if (initialArmPose!=null) {
            val zeros = controllerState.armController.boxTubeKinematics.inverse(DiffyOutputPose(initialArmPose.pivot,initialArmPose.extension))

            armEncoder1.setOffset((-armEncoder1.getRaw() + zeros.axis1.value))
            armEncoder2.setOffset((-armEncoder2.getRaw() - zeros.axis2.value))
        }

        return RobotTickI(
            controllerState,
            t = Clock.milliseconds,
            v = voltage(),
            turnEncodersPos = turnEncoders.map { it.voltage.V },
            armMotor1Pos = armEncoder1.getCounts().tick,
            armMotor2Pos = armEncoder2.getCounts().tick
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