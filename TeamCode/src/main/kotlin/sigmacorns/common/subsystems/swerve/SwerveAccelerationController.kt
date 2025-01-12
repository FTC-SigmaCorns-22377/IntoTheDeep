package sigmacorns.common.subsystems.swerve

import eu.sirotin.kotunil.base.A
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.N
import eu.sirotin.kotunil.derived.Radian
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.control.controller.Controller
import net.unnamedrobotics.lib.math.RGBA
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.Twist2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.checkedUnitless
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.polar
import net.unnamedrobotics.lib.math2.vec3
import net.unnamedrobotics.lib.math2.withZ
import net.unnamedrobotics.lib.physics.MotorPhysics
import net.unnamedrobotics.lib.physics.MotorState
import net.unnamedrobotics.lib.physics.SwerveDrivebase
import net.unnamedrobotics.lib.physics.SwerveInput
import net.unnamedrobotics.lib.physics.SwerveState
import net.unnamedrobotics.lib.rerun.RerunConnection
import net.unnamedrobotics.lib.rerun.RerunPrefix
import net.unnamedrobotics.lib.rerun.Rerunable
import net.unnamedrobotics.lib.rerun.archetypes.Arrows3D
import net.unnamedrobotics.lib.rerun.archetypes.Points3D
import sigmacorns.common.LoopTimes
import sigmacorns.common.Tuning
import sigmacorns.common.io.ControlLoopContext
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.subsystems.swerve.SwerveController.Input
import kotlin.math.absoluteValue

class SwerveAccelerationController(
    val drivebase: SwerveDrivebase
): Rerunable, Controller<SwerveAccelerationController.State, SwerveAccelerationController.Input,SwerveAccelerationController.Target>() {
    data class State(val modulePos: List<Radian>, val vel: Twist2D)
    data class Input(var turnPowers: List<Double>, var drivePowers: List<Double>)
    data class Target(var acc: Transform2D)

    override var output: Input = Input(List(4) {0.0}, List(4) { 0.0 })
    override lateinit var position: State
    override var target: Target = Target(Transform2D(0.m/s/s,0.m/s/s,0.rad/s/s))

    override fun copy(): Controller<State, Input, Target> {
        TODO("Not yet implemented")
    }

    val moduleControllers = List(4) {
        ModuleController(Tuning.SWERVE_MODULE_PID)
    }

    var swerveInput: SwerveInput = SwerveInput(List(4) { 0.rad }, List(4) { 0.m/s })

    var logPosition: Transform2D = Transform2D(0.m,0.m,0.rad)

    override fun update(deltaTime: Double): Input {
        val keepOrientation = (target.acc.vector().magnitude().value + target.acc.angle.value.absoluteValue < 0.001)

        val twist = target.acc.log()
        val motorVels = drivebase.kinematics.inverse(position.vel).drives
        val inv = drivebase.kinematics.inverse(twist)
        val motorTorques = inv.drives.map {
            val tangentialForce = it*drivebase.radius*drivebase.weight/4.0
            tangentialForce*drivebase.radius
        }

        if (!keepOrientation) swerveInput.modulePos = inv.modulePos

        val powerDriveMotors = !keepOrientation

        val drivePowers = List(swerveInput.drives.size) { i ->
            if(powerDriveMotors) tTop(drivebase.driveMotor, motorTorques[i].cast(N*m),motorVels[i].cast(rad/s)) else 0.0
        }

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
            vec3(it.first * drivebase.length / 2.0, it.second * drivebase.width / 2.0, 0.m)
        }

        val twistResolution = 10
        val twistTime = 0.5.s
        drivebase.visualize(
            SwerveState(
                List(4) { MotorState(0.A, 0.rad / s, position.modulePos[it]) },
                List(4) {
                    MotorState(
                        0.A,
                        drivebase.driveMotor.topSpeed(output.drivePowers[it]),
                        0.rad
                    )
                },
                position.vel,
                logPosition
            ), twistResolution
        )

        val moduleRadius = drivebase.moduleVizRadius
        prefix("$name/swerve") {

            log("targetTwist") {
                val locations = (0..twistResolution).map {
                    (target.acc.log() * (it / twistTime / twistResolution.toFloat())).exp().vector()
                        .withZ(0.m)
                }
                Points3D(
                    locations,
                    radii = locations.map { 2.cm.value },
                    colors = locations.map { RGBA(1.0, 0.0, 0.0, 0.0) })
            }

            val positionsVecs = List(4) {
                polar(moduleRadius, position.modulePos[it]).withZ(0.m)
            };

            output.drivePowers.forEachIndexed { i, it -> scalar("drivePowers/drivePower$i", it) }
            output.turnPowers.forEachIndexed { i, it -> scalar("turnPowers/turnPower$i", it) }

            log("turnPowerArrows") {
                Arrows3D(
                    vecs = List(4) {
                        polar(
                            output.turnPowers[it].m / moduleRadius,
                            (position.modulePos[it] + 90.degrees).cast(
                                rad
                            )
                        ).withZ(0.m)
                    },
                    origins = moduleCenters.zip(positionsVecs).map { it.first + it.second }
                )
            }

            log("targets") {
                Arrows3D(
                    vecs = List(4) {
                        polar(moduleRadius, moduleControllers[it].target.angle).withZ(0.m)
                    },
                    origins = moduleCenters,
                    colors = List(4) { RGBA(1.0, 0.0, 0.0, 1.0) }
                )
            }

            scalar("vel/vx", position.vel.dx.value)
            scalar("vel/vy", position.vel.dy.value)
            scalar("vel/vAngle", position.vel.dAngle.value)


        }
    }
}


fun tTop(motor: MotorPhysics, torque: Expression, velocity: Expression): Double
        = ((torque.cast(N*m)/motor.gearRatio/0.65 *motor.resistance/motor.torqueConstant + velocity.cast(rad/s)/motor.kV)/motor.maxVoltage).checkedUnitless()
