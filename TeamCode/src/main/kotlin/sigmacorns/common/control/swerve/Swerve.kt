package sigmacorns.common.control.swerve

import android.icu.text.DecimalFormat
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.PIDCoefficients
import net.unnamedrobotics.lib.control.controller.PIDController
import net.unnamedrobotics.lib.math.Vector2
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.PI
import kotlin.math.absoluteValue

class Swerve(
    private val driveMotors: Array<DcMotor>,
    private val turnMotors: Array<CRServo>,
    private val turnEncoders: Array<DcMotor>
) {
    private val turnPIDCoefficients = PIDCoefficients(0.00004,0.0,0.0)

    private val reversedPowers = arrayOf(1,-1,1,1)
    private val reversedEncoders = arrayOf(-1,-1,1,1)
    private val reversedTurns = arrayOf(-1,-1,-1,-1)
    private val turnDirs = (0..4).map { Vector2.fromAngle((7-it*2)* PI,1.0) }

    //set up module controllers
    private val modules: List<ModuleController>
    init {
        val moduleController = ModuleController(
            PIDController(turnPIDCoefficients),
            0.0,
            ModuleTarget(Vector2(0,0),false)
        )

        modules = (0..4).map { i -> moduleController.copy().also { module ->
            module.drivePowerReversed = reversedPowers[i].toDouble()
            module.reverseEncoders = reversedEncoders[i].toDouble()
            module.turnPowerReversed = reversedTurns[i].toDouble()
        } }
    }

    private var vs = (0..4).map { Vector2() }
    fun update(transform: Vector2, turn: Double, lockWheels: Boolean, dt: Double) {
        val keepOrientation = (transform.magnitude + turn.absoluteValue < 0.001)
        if (!keepOrientation) vs = turnDirs.map { it*turn + transform }
        if (lockWheels) vs = turnDirs.map { it }
        val powerDriveMotors = !(lockWheels || keepOrientation)

        val maxMag = vs.maxBy { it.magnitude }.magnitude
        if(maxMag>1.0) vs = vs.map { it/maxMag }

        modules.mapIndexed { i, module ->
            module target  ModuleTarget(v=vs[i], powerDriveMotors)
            module updatePosition reversedEncoders[i]*turnEncoders[i].currentPosition.toDouble()
            module.update(dt).let {
                turnMotors[i].power = it.turn
                driveMotors[i].power = it.drive
            }
        }
    }

    fun reset() {
        modules.forEach {
            it.target = ModuleTarget(v=Vector2(0,0), powerDriveMotors = false)
            it.position=0.0
        }

        turnEncoders.forEach {
            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }

    private var telemetry: Telemetry? = null
    fun telemetry(tel: Telemetry) {
        telemetry = tel
        for(i in modules.indices) {
            val fmt = DecimalFormat("#,###.##")
            val caption = "Wheel " + (i+1) + ": "
            tel.addData(caption + "target angle = ", fmt.format(modules[i].target.v.angleFromOrigin))
            tel.addData(caption + "pos = ", fmt.format(modules[i].position))
            tel.addData(caption + "power = ",  fmt.format(modules[i].position))
            tel.addData(caption + "angle = ", fmt.format(tickToAngle((reversedEncoders[i]*turnEncoders[i].currentPosition))))
            tel.addData("vs [$i]",vs[i])
        }

    }
}