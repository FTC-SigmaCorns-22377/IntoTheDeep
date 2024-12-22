package sigmacorns.common.subsystems.swerve

import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.core.div
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.control.controller.Controller
import net.unnamedrobotics.lib.control.controller.PIDController
import net.unnamedrobotics.lib.control.controller.params.PIDCoefficients
import net.unnamedrobotics.lib.math.Vector2
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.polar
import net.unnamedrobotics.lib.math2.vec3
import net.unnamedrobotics.lib.math2.withZ
import net.unnamedrobotics.lib.math2.plus
import net.unnamedrobotics.lib.rerun.RerunConnection
import net.unnamedrobotics.lib.rerun.RerunPrefix
import net.unnamedrobotics.lib.rerun.Rerunable
import net.unnamedrobotics.lib.rerun.archetypes.Arrows3D
import net.unnamedrobotics.lib.rerun.archetypes.Ellipsoids3D
import net.unnamedrobotics.lib.rerun.archetypes.FillMode
import sigmacorns.common.Tuning
import kotlin.math.PI
import kotlin.math.absoluteValue

typealias SwerveState = List<Double>
data class SwerveInput(var turnPowers: Array<Double>, var drivePowers: Array<Double>)
data class SwerveTarget(var transform: Vector2, var turn: Double, var lockWheels: Boolean)

class SwerveController(
    turnPIDCoefficients: PIDCoefficients = Tuning.SWERVE_MODULE_PID
): Controller<SwerveState, SwerveInput, SwerveTarget>(), Rerunable {
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
    override var output: SwerveInput = SwerveInput(Array(4) { 0.0 }, Array(4) { 0.0 })

    override var position = listOf(0.0,0.0,0.0,0.0)
    override var target = SwerveTarget(Vector2(),0.0,false)

    override fun update(dt: Double): SwerveInput {
        val keepOrientation = (target.transform.magnitude + target.turn.absoluteValue < 0.001)
        if (!keepOrientation) vs = turnDirs.map { it*target.turn + target.transform }
        if (target.lockWheels) vs = turnDirs.map { it }
        val powerDriveMotors = !(target.lockWheels || keepOrientation)

        val maxMag = vs.maxBy { it.magnitude }.magnitude
        if(maxMag>1.0) vs = vs.map { it/maxMag }

        return modules.foldIndexed(SwerveInput(arrayOf(),arrayOf())) { i, acc, module ->
            module.target = ModuleTarget(v=vs[i], powerDriveMotors)
            module.position = position[i]
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

    override fun copy(): Controller<SwerveState, SwerveInput, SwerveTarget> {
        TODO()
//        return SwerveController()
    }

    fun logRerun(connection: RerunConnection) {
//        rerun(connection) {
    }

    context(RerunPrefix, RerunConnection) override fun log(name: String) {
        val moduleCenters = List(4) { i -> polar(1.m, (45+i*90).degrees).withZ(0.m) }
        val moduleRadius = 0.1.m

        prefix(name) {

            log("modules") {
                Ellipsoids3D(
                    centers = moduleCenters,
                    sizes = List(4) { vec3(0.25,0.25,0.0) },
                    fillMode = FillMode.MajorWireframe,
                )
            }

            log("drive powers") {
                Arrows3D(
                    vecs = List(4) {
                        polar((output.drivePowers[it]/moduleRadius),position[it].rad).withZ(0.m)
                    },
                    origins = moduleCenters
                )
            }


            val positionsVecs = List(4) {
                polar(moduleRadius,position[it].rad).withZ(0.m)
            };

            log("positions") {
                Arrows3D(
                    vecs = positionsVecs,
                    origins = moduleCenters
                )
            }

            log("turn powers") {
                Arrows3D(
                    vecs = List(4) {
                        polar(output.turnPowers[it].m/moduleRadius,position[it].rad+90.degrees).withZ(0.m)
                    },
                    origins = moduleCenters.zip(positionsVecs).map { it.first+it.second }
                )
            }

            log("targets") {
                Arrows3D(
                    vecs = List(4) {
                        polar(moduleRadius,modules[it].target.v.angleFromOrigin.rad).withZ(0.m)
                    },
                    origins = moduleCenters
                )
            }
        }
    }
}