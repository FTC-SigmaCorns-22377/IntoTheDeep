package sigmacorns.common.swerve

import android.icu.text.DecimalFormat
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.PIDCoefficients
import net.unnamedrobotics.lib.control.controller.GeneralController
import net.unnamedrobotics.lib.control.controller.PIDController
import net.unnamedrobotics.lib.math.Vector2
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.PI
import kotlin.math.absoluteValue

typealias SwerveState = Array<Double>
data class SwerveInput(var turnPowers: Array<Double>, var drivePowers: Array<Double>)
data class SwerveTarget(var transform: Vector2, var turn: Double, var lockWheels: Boolean)

class SwerveController: GeneralController<SwerveState,SwerveInput,SwerveTarget>() {
    private val turnPIDCoefficients = PIDCoefficients(0.00004,0.0,0.0)

    private val turnDirs = Array(4){ Vector2.fromAngle((7-it*2)/4.0* PI,1.0) }

    //set up module controllers
    val modules: List<ModuleController>
    init {
        val moduleController = ModuleController(
            PIDController(turnPIDCoefficients),
            0.0,
            ModuleTarget(Vector2(0,0),false)
        )

        modules = List(4) { moduleController.copy() }
    }

    private var vs = List(4) { Vector2() }
    val moduleTargetVectors
        get() = vs
    override var position = arrayOf(0.0,0.0,0.0,0.0)
    override var target = SwerveTarget(Vector2(),0.0,false)

    override fun update(dt: Double): SwerveInput {
        val keepOrientation = (target.transform.magnitude + target.turn.absoluteValue < 0.001)
        if (!keepOrientation) vs = turnDirs.map { it*target.turn + target.transform }
        if (target.lockWheels) vs = turnDirs.map { it }
        val powerDriveMotors = !(target.lockWheels || keepOrientation)

        val maxMag = vs.maxBy { it.magnitude }.magnitude
        if(maxMag>1.0) vs = vs.map { it/maxMag }

        return modules.foldIndexed(SwerveInput(arrayOf(),arrayOf())) { i, acc, module ->
            module target  ModuleTarget(v=vs[i], powerDriveMotors)
            module updatePosition position[i]
            module.update(dt).let {
                acc.turnPowers += it.turn
                acc.drivePowers += it.drive
            }
            acc
        }
    }

    fun resetEncoders() {
        modules.forEach {
            it.target = ModuleTarget(v=Vector2(0,0), powerDriveMotors = false)
            it.position=0.0
        }
    }

    override fun copy(): GeneralController<SwerveState, SwerveInput, SwerveTarget> {
        return SwerveController()
    }
}