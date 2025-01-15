package sigmacorns.common.subsystems.swerve

import eu.sirotin.kotunil.base.A
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.derived.Radian
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.control.controller.Controller
import net.unnamedrobotics.lib.math.RGB
import net.unnamedrobotics.lib.math.RGBA
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.Twist2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.orthogonal
import net.unnamedrobotics.lib.math2.polar
import net.unnamedrobotics.lib.math2.vec2
import net.unnamedrobotics.lib.math2.vec3
import net.unnamedrobotics.lib.math2.withZ
import net.unnamedrobotics.lib.physics.MotorState
import net.unnamedrobotics.lib.physics.SwerveDrivebase
import net.unnamedrobotics.lib.physics.SwerveInput
import net.unnamedrobotics.lib.physics.SwerveState
import net.unnamedrobotics.lib.rerun.RerunConnection
import net.unnamedrobotics.lib.rerun.RerunPrefix
import net.unnamedrobotics.lib.rerun.Rerunable
import net.unnamedrobotics.lib.rerun.archetypes.Arrows3D
import net.unnamedrobotics.lib.rerun.archetypes.Points3D
import kotlin.math.absoluteValue

class SwerveController(
    val module: ModuleController,
    val drivebase: SwerveDrivebase,
): Rerunable, Controller<SwerveController.State,SwerveController.Input,SwerveController.Target>() {
    data class State(val modulePos: List<Radian>)
    data class Input(var turnPowers: List<Double>, var drivePowers: List<Double>)
    data class Target(var vel: Transform2D, var lockWheels: Boolean)

    override var output: Input = Input(List(4) { 0.0 }, List(4) { 0.0 })
    override lateinit var position: State
    override var target: Target = Target(Transform2D(0.m/s,0.m/s,0.rad/s),false)

    val moduleControllers = List(4) { module.copy() }
    var swerveInput: SwerveInput = SwerveInput(List(4) { 0.rad }, List(4) { 0.m/s })

    var logPosition: Transform2D = Transform2D(0.m,0.m,0.rad)
    var logVelocity: Twist2D = Twist2D(0.m/s,0.m/s,0.rad/s)

    override fun copy() = SwerveController(module, drivebase)

    override fun update(deltaTime: Double): Input {
        val keepOrientation = (target.vel.vector().magnitude().value + target.vel.angle.value.absoluteValue < 0.001)

        val twist = target.vel.log()

        if (!keepOrientation) swerveInput = drivebase.let { s ->
            val vs = s.wheels.map { wheel ->
                twist.vector() - wheel.position.orthogonal().normalized()*wheel.position.magnitude() * twist.dAngle
            }

            SwerveInput(
                vs.map { it.theta() },
                vs.map { it.magnitude()/drivebase.radius }
            )
//            drivebase.kinematics.inverse(twist)
        }

        if (target.lockWheels) {
            val x = drivebase.kinematics.inverse(Twist2D(0.m/s,0.m/s,1.rad/s))
            swerveInput = SwerveInput(x.modulePos.map { (it + 90.degrees).cast(rad) },List(4) { 0.rad/s })
        }

        val powerDriveMotors = !(target.lockWheels || keepOrientation)

        val drivePowers = swerveInput.drives.map { if(powerDriveMotors) drivebase.driveMotor.velToPower(it) else 0.0 }

        moduleControllers.forEachIndexed { i,it ->
            it.target = ModuleTarget(swerveInput.modulePos[i].cast(rad), drivePowers[i])
            it.position = position.modulePos[i]

            // TODO: update lib to autoassign to output
            it.output = it.update(deltaTime)
        }

        return Input(
            moduleControllers.map { it.output.turn },
            moduleControllers.map { it.output.drive }
        )
    }

    context(RerunPrefix, RerunConnection) override fun log(name: String) {
        val moduleCenters = listOf(1.0 to 1.0, 1.0 to -1.0, -1.0 to 1.0, -1.0 to -1.0).map {
            vec3(it.first * drivebase.length / 2.0, it.second * drivebase.width / 2.0,0.m)
        }

        val twistResolution = 10
        val twistTime = 0.5.s

        // TODO: make this accept a name in library and implement rerunnable. also customizable colors :)
        val moduleRadius = drivebase.moduleVizRadius
        prefix("$name/visual") {
            drivebase.visualize(SwerveState(
                List(4) { MotorState(0.A,0.rad/s, position.modulePos[it]) },
                List(4) { MotorState(0.A, drivebase.driveMotor.topSpeed(output.drivePowers[it]), 0.rad) },
                logVelocity,
                logPosition
            ),twistResolution)

            val positionsVecs = List(4) {
                polar(moduleRadius,position.modulePos[it]).withZ(0.m)
            };

            log("targetTwist") {
                val locations = (0..twistResolution).map { (target.vel.log()*(it/twistTime/twistResolution.toFloat())).exp().vector().withZ(0.m) }
                Points3D(locations, radii = locations.map { 2.cm.value }, colors = locations.map { RGBA(1.0,0.0,0.0,0.0) })
            }

            log("targetVel") {
                Arrows3D(
                    origins = listOf(vec3(0.m,0.m,0.m)),
                    vecs = listOf(target.vel.vector().withZ(0.m))
                )
            }

            log("turnPowerArrows") {
                Arrows3D(
                    vecs = List(4) {
                        polar(output.turnPowers[it].m*moduleRadius,(position.modulePos[it]+90.degrees).cast(
                            rad
                        )).withZ(0.m)
                    },
                    origins = moduleCenters.zip(positionsVecs).map { it.first+it.second }
                )
            }

            log("targets") {
                Arrows3D(
                    vecs = List(4) {
                        polar(moduleRadius,moduleControllers[it].target.angle).withZ(0.m)
                    },
                    origins = moduleCenters,
                    colors = List(4) { RGBA(1.0,0.0,0.0,1.0) }
                )
            }
        }

        prefix(name) {
            output.drivePowers.forEachIndexed { i,it -> scalar("drivePowers/drivePower$i",it) }
            output.turnPowers.forEachIndexed { i,it -> scalar("turnPowers/turnPower$i",it) }

            position.modulePos.forEachIndexed { i, it -> scalar("modulePositions/$i", it.value) }

            scalar("vel/vx",logVelocity.dx.value)
            scalar("vel/vy",logVelocity.dy.value)
            scalar("vel/vAngle",logVelocity.dAngle.value)

            scalar("desVel/vx",target.vel.x.value)
            scalar("desVel/vy",target.vel.y.value)
            scalar("desVel/vAngle",target.vel.angle.value)
        }
    }
}