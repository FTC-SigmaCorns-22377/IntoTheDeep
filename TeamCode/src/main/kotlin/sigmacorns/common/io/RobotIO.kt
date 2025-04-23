package sigmacorns.common.io

import android.graphics.Color
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import eu.sirotin.kotunil.base.A
import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.V
import eu.sirotin.kotunil.derived.Volt
import net.unnamedrobotics.lib.driver.gobilda.GoBildaPinpointDriver
import net.unnamedrobotics.lib.math2.Tick
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.Twist2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.tick
import net.unnamedrobotics.lib.rerun.RerunConnection
import net.unnamedrobotics.lib.rerun.RerunSources
import net.unnamedrobotics.lib.util.Clock
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import sigmacorns.constants.Debug

class RobotIO(
    hardwareMap: HardwareMap,
    val rerunSrc: RerunSources = RerunSources.TCP("192.168.43.122"),
    val rerunName: String = "unnamed",
    initialPos: Transform2D? = null
): SigmaIO() {
    private var rerun: RerunConnection? = null
    override val rerunConnection: RerunConnection
        get() = rerun ?: RerunConnection(rerunName, rerunSrc).also { rerun = it }

    private val m1: DcMotor? = hardwareMap.tryGet(DcMotor::class.java,"M3")
    private val m2: DcMotor? = hardwareMap.tryGet(DcMotor::class.java,"M1")
    private val m3: DcMotor? = hardwareMap.tryGet(DcMotor::class.java,"M2")

    private val fl: DcMotor? = hardwareMap.tryGet(DcMotor::class.java,"FL")
    private val bl: DcMotor? = hardwareMap.tryGet(DcMotor::class.java,"BL")
    private val br: DcMotor? = hardwareMap.tryGet(DcMotor::class.java,"BR")
    private val fr: DcMotor? = hardwareMap.tryGet(DcMotor::class.java,"FR")

    private val mIntake: DcMotor? = hardwareMap.tryGet(DcMotor::class.java,"M0")

    private val sArmL: Servo? = hardwareMap.tryGet(Servo::class.java, "AL")
    private val sArmR: Servo? = hardwareMap.tryGet(Servo::class.java, "AR")
    private val sWrist: Servo? = hardwareMap.tryGet(Servo::class.java, "wrist")

    private val sClaw: Servo? = hardwareMap.tryGet(Servo::class.java, "claw")
    private val sPush: Servo? = hardwareMap.tryGet(Servo::class.java,"push")
    private val sFlap: Servo? = hardwareMap.tryGet(Servo::class.java, "S4")

    private val t1: Servo? = hardwareMap.tryGet(Servo::class.java,"S1C")
    private val t2: Servo? = hardwareMap.tryGet(Servo::class.java,"S2C")

    private val sPto1: Servo? = hardwareMap.tryGet(Servo::class.java,"PTO1")
    private val sPto2: Servo? = hardwareMap.tryGet(Servo::class.java,"PTO2")

    private val colorSensor: ColorRangeSensor? = hardwareMap.tryGet(ColorRangeSensor::class.java, "color")

    val limelight: Limelight3A? = hardwareMap.tryGet(Limelight3A::class.java, "limelight")

    val imu: IMU? = hardwareMap.tryGet(IMU::class.java, "imu")

    private val pinpointDriver: GoBildaPinpointDriver? = hardwareMap.tryGet(
        GoBildaPinpointDriver::class.java,"pinpoint")
    private val pinpoint = pinpointDriver?.let { PinpointLocalizer(it) }

    private val intakeTouch = hardwareMap.tryGet(TouchSensor::class.java,"intakeTouch")
    private val liftTouch = hardwareMap.tryGet(TouchSensor::class.java,"liftTouch")

    init {
        fl?.direction = DcMotorSimple.Direction.REVERSE
        bl?.direction = DcMotorSimple.Direction.REVERSE

        val driveMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        fl?.mode = driveMode
        fr?.mode = driveMode
        bl?.mode = driveMode
        br?.mode = driveMode

        m1?.direction = DcMotorSimple.Direction.REVERSE
        m2?.direction = DcMotorSimple.Direction.FORWARD
        m3?.direction = DcMotorSimple.Direction.REVERSE

        m1?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        m2?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        m3?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        m1?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        m2?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        m3?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        colorSensor?.argb()

        pinpointDriver?.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        if(initialPos!=null) pinpointDriver?.setPosition(transToPose(initialPos))

        if(!Debug.ALLOW_INCOMPLETE_HARDWARE_MAP) {
            m1!!
            m2!!
            fl!!
            fr!!
            bl!!
            br!!
            mIntake!!
            sArmL!!
            sArmR!!
            sClaw!!
            sPush!!
            sFlap!!
            t1!!
            t2!!
            sPto1!!
            sPto2!!
            colorSensor!!
            limelight!!
            imu!!
            pinpoint!!
            sWrist!!
        }
    }

    private fun transToPose(t: Transform2D): Pose2D
        = Pose2D(
                DistanceUnit.METER,
                t.x.value,t.y.value,
                AngleUnit.RADIANS,
                t.angle.value
            )

    override fun setPinPos(p: Transform2D) {
        pinpointDriver?.setPosition(transToPose(p).let {
            Pose2D(
                DistanceUnit.CM,
                -it.getY(DistanceUnit.CM),
                it.getX(DistanceUnit.CM),
                AngleUnit.RADIANS,
                it.getHeading(AngleUnit.RADIANS)
            )
        })
    }

    private var m1Bias = 0.tick
    private var m2Bias = 0.tick

    override fun resetSlideMotors(zero1: Tick, zero2: Tick) {
        m1Bias = (-lastM1Pos+zero1).cast(tick)
        m2Bias = (-lastM2Pos+zero2).cast(tick)
    }

    private var lastColorVal: Int = 0
    private var distance: Metre = 0.m
    private var lastM1Pos = 0.tick
    private var lastM2Pos = 0.tick
    private var lastVoltage = 12.V
    private var wasIntakeTriggered = false
    private var wasLiftTriggered = false

    override fun updateSensors() {
         lastColorVal = colorSensor?.argb() ?: 0
        distance = (colorSensor?.getDistance(DistanceUnit.METER) ?: 0).m
        pinpoint?.update(true)
        lastM1Pos = (m1?.currentPosition ?: 0).tick
        lastM2Pos = (m2?.currentPosition ?: 0).tick
        wasIntakeTriggered = intakeTouch?.isPressed ?: false
        wasLiftTriggered = liftTouch?.isPressed ?: false
    }

    override fun position() = (pinpoint?.getTransform() ?: Transform2D(0, 0, 0)).let {
        Transform2D(it.y,-it.x,it.angle)
    }
    override fun velocity() = (pinpoint?.velocity ?: Twist2D(0, 0, 0)).let {
        Twist2D(it.dy,-it.dx,it.dAngle)
    }

    override fun motor1Pos() = (lastM1Pos + m1Bias).cast(tick)
    override fun motor2Pos() = (lastM2Pos + m2Bias).cast(tick)
    override fun red() = Color.red(lastColorVal)
    override fun green() = Color.green(lastColorVal)
    override fun blue() = Color.blue(lastColorVal)
    override fun alpha() = Color.alpha(lastColorVal)
    override fun distance(): Metre = distance
    override fun voltage(): Volt = lastVoltage
    override fun time() = Clock.seconds.s
    override fun intakeLimitTriggered(): Boolean = wasIntakeTriggered
    override fun liftLimitTriggered(): Boolean = wasLiftTriggered
    override fun intakeCurrent(): Expression
        = (mIntake as DcMotorEx).getCurrent(CurrentUnit.AMPS).A

    override var driveFL: Double = 0.0
        set(value) {
            if(value!=field) {
                fl?.power = value
                field = value
            }
        }
    override var driveBL: Double = 0.0
        set(value) {
            if(value!=field) {
                bl?.power = value
                field = value
            }
        }
    override var driveBR: Double = 0.0
        set(value) {
            if(value!=field) {
                br?.power = value
                field = value
            }
        }

    override var driveFR: Double = 0.0
        set(value) {
            if(value!=field) {
                fr?.power = value
                field = value
            }
        }

    override var motor1: Double = 0.0
        set(value) {
            if(value!=field) {
                m1?.power = value
                field = value
            }
        }

    override var motor2: Double = 0.0
        set(value) {
            if(value!=field) {
                m2?.power = value
                field = value
            }
        }
    override var motor3: Double = 0.0
        set(value) {
            if(value!=field) {
                m3?.power = value
                field = value
            }
        }

    override var intake: Double = 0.0
        set(value) {
            if(value!=field) {
                mIntake?.power = value
                field = value
            }
        }
    override var push: Double = 0.0
        set(value) {
            if (value != field) {
                sPush?.position = value
                field = value
            }
        }
    override var armL: Double = 0.0
        set(value) {
            if(value!=field) {
                sArmL?.position = value
                field = value
            }
        }
    override var armR: Double = 0.0
        set(value) {
            if(value!=field) {
                sArmR?.position = value
                field = value
            }
        }
    override var wrist: Double = 0.0
        set(value) {
            if(value!=field) {
                sWrist?.position = value
                field = value
            }
        }
    override var claw: Double = 0.0
        set(value) {
            if(value!=field) {
                sClaw?.position = value
                field = value
            }
        }
    override var flap: Double = 0.0
        set(value) {
            if(value!=field) {
                sFlap?.position = value
                field = value
            }
        }
    override var tilt1: Double = 0.0
        set(value) {
            if(value!=field) {
                t1?.position = value
                field = value
            }
        }
    override var tilt2: Double = 0.0
        set(value) {
            if(value!=field) {
                t2?.position = 1-value
                field = value
            }
        }
    override var pto1: Double = 0.0
        set(value) {
            if(value!=field) {
                sPto1?.position = value
                field = value
            }
        }
    override var pto2: Double = 0.0
        set(value) {
            if(value!=field) {
                sPto2?.position = value
                field = value
            }
        }
}