package sigmacorns.common.hardware

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU

class RobotHardware(hardwareMap: HardwareMap) {
    
    object:T  object::class: KClass<T>
    val drive1 = hardwareMap.get(DcMotor::class.java, "m1")
    val drive2 = hardwareMap.get(DcMotor::class.java, "m2")
    val drive3 = hardwareMap.get(DcMotor::class.java, "m3")
    val drive4 = hardwareMap.get(DcMotor::class.java, "m4")
    val drives = listOf(drive1,drive2,drive3,drive4)

    val turn1 = hardwareMap.get(CRServo::class.java, "t1")
    val turn2 = hardwareMap.get(CRServo::class.java, "t3")
    val turn3 = hardwareMap.get(CRServo::class.java, "t2")
    val turn4 = hardwareMap.get(CRServo::class.java, "t4")
    val turns = listOf(drive1,drive2,drive3,drive4)

    val octo = hardwareMap.get(OctoQuad::class.java, "octo")

    val imu = hardwareMap.get(IMU::class.java,"imu")

    init {
        val params = IMU.Parameters(
            RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            )
        )
        imu.initialize(params)
    }
}