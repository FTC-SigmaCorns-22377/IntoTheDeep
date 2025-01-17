package sigmacorns.common.subsystems.swerve

import eu.sirotin.kotunil.base.Second
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.Radian
import eu.sirotin.kotunil.derived.V
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.control.controller.Controller
import net.unnamedrobotics.lib.control.controller.PIDController
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.Vector2
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.map
import net.unnamedrobotics.lib.math2.normalizeRadian
import net.unnamedrobotics.lib.math2.revolution
import net.unnamedrobotics.lib.math2.vec2
import net.unnamedrobotics.lib.physics.SwerveDrivebase
import sigmacorns.common.Constants.MODULE_OFFSET
import sigmacorns.common.LOGGING
import sigmacorns.common.LoopTimes
import sigmacorns.common.Tuning
import sigmacorns.common.io.ControlLoopContext
import sigmacorns.common.io.SigmaIO
import kotlin.math.absoluteValue


class SwerveHeadingController(controller: SwerveController):
    Controller<SwerveHeadingController.State, SwerveController.Input, SwerveHeadingController.Target>() {
    data class Target(val velocity: Vector2, val angle: Radian)
    data class State(val heading: Radian, val modulePos: List<Radian>)

    override var output: SwerveController.Input = SwerveController.Input(List(4) { 0.0 }, List(4) { 0.0 })
    override var position = State(0.rad, List(4) { 0.0.rad })
    override var target: Target = Target(vec2(0.m/s, 0.m/s), 0.rad)

    val swerveController = controller
    val turnController = PIDController(Tuning.SWERVE_HEADING_PID)

    override fun copy(): Controller<State, SwerveController.Input, Target> {
        TODO("Not yet implemented")
    }


    var profileT = 0.s
    var profile: ((Second) -> Expression)? = null

    override fun update(deltaTime: Double): SwerveController.Input {
        turnController.coefficients = Tuning.SWERVE_HEADING_PID
        val err = (target.angle - position.heading).normalizeRadian()
        var theta = target.angle

        if(err.map { it.absoluteValue } > Tuning.SWERVE_HEADING_I_ZONE) {
            profile = Tuning.SWERVE_HEADING_PROFILE.new(position.heading,target.angle)
            profileT = 0.s

            theta = (profile!!)(profileT).cast(rad)
            profileT = (profileT + deltaTime.s).cast(s)
            turnController.integral = 0.0
        }

        val diff = (theta-position.heading).normalizeRadian()
        val turnSpeed = turnController.updateStateless(deltaTime,position.heading.value,target.angle.value).rad/s

        return swerveController.updateStateless(deltaTime,SwerveController.State(position.modulePos),
            SwerveController.Target(Transform2D(
                target.velocity,turnSpeed
            ),false))
    }
}
fun swerveHeadingControlLoop(controller: SwerveController) = ControlLoopContext(
    LoopTimes.SWERVE,
    SwerveHeadingController(controller),
    { io: SigmaIO ->
        SwerveHeadingController.State(
            io.position().angle.cast(rad),
            io.turnVoltages()
                .map { (it/(3.3.V)* revolution) }
                .zip(MODULE_OFFSET)
                .map { (it.first - it.second).cast(rad) }
        )
    },
    { u: SwerveController.Input, io: SigmaIO ->
        io.drivePowers.zip(u.drivePowers).forEach { it.first.write(it.second) }
        io.turnPowers.zip(u.turnPowers).forEach { it.first.write(it.second) }
    },
    { if(LOGGING.RERUN_SWERVE) controller.log("swerve") },
    name = "swerve"
)