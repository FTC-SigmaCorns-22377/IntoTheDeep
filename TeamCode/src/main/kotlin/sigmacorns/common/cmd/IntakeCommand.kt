package sigmacorns.common.cmd

import com.qualcomm.robotcore.util.ElapsedTime
import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.Second
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.ms
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.microbecquerel
import net.unnamedrobotics.lib.command.Command
import net.unnamedrobotics.lib.command.Status
import net.unnamedrobotics.lib.command.cmd
import net.unnamedrobotics.lib.command.groups.parallel
import net.unnamedrobotics.lib.command.groups.plus
import net.unnamedrobotics.lib.command.groups.then
import net.unnamedrobotics.lib.command.schedule
import sigmacorns.common.Robot
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.constants.Color
import sigmacorns.constants.Tuning

fun extendCommand(robot: Robot, dist: Metre) = robot.extendCommandSlot.register(robot.slides.follow {
    DiffyOutputPose(dist, robot.slides.t.axis2)
})

fun powerIntakeCommand(robot: Robot, power: Double) = cmd {
    instant { robot.io.intake = power }
}

fun intakeCommand(robot: Robot, dist: Metre) =
    extendCommand(robot,dist) +
    powerIntakeCommand(robot, Tuning.ACTIVE_POWER) +
    robot.intake.follow(Tuning.IntakePosition.ACTIVE)

//fun detectCommand(robot: Robot) = cmd {
//    finishWhen { robot.io.distance() < Color.DIST_THRESHOLD || }
//}

// TODO: onCancel!!!!

fun brakeIntakeRollers(robot: Robot) =
    instant { robot.active.updatePort(Tuning.ACTIVE_STOP_POWER) } then
    wait(Tuning.ACTIVE_STOP_TIME) then
    instant { robot.active.updatePort(0.0) }

val MIN_AUTO_INTAKE_TIME = 300.ms
fun autoIntake(robot: Robot, dist: Metre) = deadline(cmd {
    var expireTime: Expression = 0.s
    init { expireTime = robot.io.time() + MIN_AUTO_INTAKE_TIME}
    loop {
        if(robot.io.time() > expireTime && robot.slides.t.axis1<10.cm) status = Status.CANCELLED
    }
    finishWhen { robot.io.distance() < Color.DIST_THRESHOLD }
}, intakeCommand(robot,dist)) then brakeIntakeRollers(robot) then transferCommand(robot)

fun retract(robot: Robot) =
    parallel(extendCommand(robot,0.m), powerIntakeCommand(robot,0.0), robot.intake.follow(Tuning.IntakePosition.OVER))

fun wait(t: Second) = cmd {
    val curTime = ElapsedTime()
    init { curTime.reset() }
    finishWhen { curTime.seconds().s > t }
}