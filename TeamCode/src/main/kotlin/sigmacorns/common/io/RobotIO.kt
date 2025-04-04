package sigmacorns.common.io

import android.graphics.Color
import android.graphics.ColorSpace
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.V
import eu.sirotin.kotunil.derived.Volt
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.driver.gobilda.GoBildaPinpointDriver
import net.unnamedrobotics.lib.math.Pose
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.Twist2D
import net.unnamedrobotics.lib.math2.tick
import net.unnamedrobotics.lib.rerun.RerunConnection
import net.unnamedrobotics.lib.util.Clock
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D

class RobotIO(
    hardwareMap: HardwareMap,
    val ip: String = "192.168.43.122",
    val rerunName: String = "unnamed",
    initialPos: Transform2D? = null
): SigmaIO() {
    private var rerun: RerunConnection? = null
    override val rerunConnection: RerunConnection
        get() = rerun ?: RerunConnection(rerunName, ip).also { rerun = it }

    private val m1 = hardwareMap.get(DcMotor::class.java,"D1")
    private val m2 = hardwareMap.get(DcMotor::class.java,"D2")

    private val fl = hardwareMap.get(DcMotor::class.java,"FL")
    private val bl = hardwareMap.get(DcMotor::class.java,"BL")
    private val br = hardwareMap.get(DcMotor::class.java,"BR")
    private val fr = hardwareMap.get(DcMotor::class.java,"FR")

    private val mIntake = hardwareMap.get(DcMotor::class.java,"intake")

    private val sArmL = hardwareMap.get(Servo::class.java, "AL")
    private val sArmR = hardwareMap.get(Servo::class.java, "AR")
    private val sClaw = hardwareMap.get(Servo::class.java, "claw")
    private val sIntake1 = hardwareMap.get(Servo::class.java, "I1")
    private val sIntake2 = hardwareMap.get(Servo::class.java, "I2")
    private val sFlap = hardwareMap.get(Servo::class.java, "flap")

    private val t1 = hardwareMap.get(Servo::class.java,"T1")
    private val t2 = hardwareMap.get(Servo::class.java,"T2")

    private val colorSensor = hardwareMap.get(ColorRangeSensor::class.java, "color")

    val limelight = hardwareMap.get(Limelight3A::class.java, "limelight")

    val imu = hardwareMap.get(IMU::class.java, "imu")

    private val pinpointDriver = hardwareMap.get(
        GoBildaPinpointDriver::class.java,"pinpoint")

    private val pinpoint = PinpointLocalizer(pinpointDriver)

    init {
        fl.direction = DcMotorSimple.Direction.REVERSE
        bl.direction = DcMotorSimple.Direction.REVERSE
        m1.direction = DcMotorSimple.Direction.FORWARD
        m2.direction = DcMotorSimple.Direction.REVERSE

        m1.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        m2.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        m1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        m2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        colorSensor.argb()

        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        if(initialPos!=null) pinpointDriver.setPosition(transToPose(initialPos))
    }

    private fun transToPose(t: Transform2D): Pose2D
        = Pose2D(
                DistanceUnit.METER,
                t.x.value,t.y.value,
                AngleUnit.RADIANS,
                t.angle.value
            )

    override fun setPinPos(p: Transform2D) {
        pinpointDriver.setPosition(transToPose(p))
    }

    override fun updatePinpoint() {
        pinpoint.update(true)
    }

    override fun position() = pinpoint.getTransform()
    override fun velocity() = pinpoint.velocity
    override fun motor1Pos() = m1.currentPosition.tick
    override fun motor2Pos() = m2.currentPosition.tick

    private var lastColorVal: Int = 0
    private var distance: Metre = 0.m

    override fun updateColorDist() {
        lastColorVal = colorSensor.argb()
        distance = colorSensor.getDistance(DistanceUnit.METER).m
    }

    override fun red() = Color.red(lastColorVal)
    override fun green() = Color.green(lastColorVal)
    override fun blue() = Color.blue(lastColorVal)
    override fun alpha() = Color.alpha(lastColorVal)
    override fun distance(): Metre = distance

    override fun voltage(): Volt = 12.V

    override fun time() = Clock.seconds.s

    override var driveFL: Double = 0.0
        set(value) {
            if(value!=field) {
                fl.power = value
                field = value
            }
        }
    override var driveBL: Double = 0.0
        set(value) {
            if(value!=field) {
                bl.power = value
                field = value
            }
        }
    override var driveBR: Double = 0.0
        set(value) {
            if(value!=field) {
                br.power = value
                field = value
            }
        }

    override var driveFR: Double = 0.0
        set(value) {
            if(value!=field) {
                fr.power = value
                field = value
            }
        }

    override var motor1: Double = 0.0
        set(value) {
            if(value!=field) {
                m1.power = value
                field = value
            }
        }

    override var motor2: Double = 0.0
        set(value) {
            if(value!=field) {
                m2.power = value
                field = value
            }
        }

    override var intake: Double = 0.0
        set(value) {
            if(value!=field) {
                mIntake.power = value
                field = value
            }
        }

    override var intakeL: Double = 0.0
        set(value) {
            if(value!=field) {
                sIntake1.position = 1.0-value
                field = value
            }
        }
    override var intakeR: Double = 0.0
        set(value) {
            if(value!=field) {
                sIntake2.position = 1.0-value
                field = value
            }
        }

    override var armL: Double = 0.0
        set(value) {
            if(value!=field) {
                sArmL.position = value
                field = value
            }
        }
    override var armR: Double = 0.0
        set(value) {
            if(value!=field) {
                sArmR.position = value
                field = value
            }
        }
    override var claw: Double = 0.0
        set(value) {
            if(value!=field) {
                sClaw.position = value
                field = value
            }
        }
    override var flap: Double = 0.0
        set(value) {
            if(value!=field) {
                sFlap.position = value
                field = value
            }
        }
    override var tilt1: Double = 0.0
        set(value) {
            if(value!=field) {
                t1.position = value
                field = value
            }
        }
    override var tilt2: Double = 0.0
        set(value) {
            if(value!=field) {
                t2.position = value
                field = value
            }
        }
}