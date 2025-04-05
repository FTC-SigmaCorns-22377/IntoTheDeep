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
import net.unnamedrobotics.lib.command.groups.series
import net.unnamedrobotics.lib.command.groups.then
import sigmacorns.common.Robot
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.constants.Color
import sigmacorns.constants.Tuning

fun extendCommand(robot: Robot, dist: Metre, lock: Boolean = true) = (robot.slides.follow {
    DiffyOutputPose(dist, robot.slides.t.axis2)
})
    .let { if(lock) robot.extendCommandSlot.register(it) else it }
    .name("extendCommand($dist)")

fun powerIntakeCommand(robot: Robot, power: Double) = cmd {
    var old: Double? = null;
    instant {
        old = robot.active
        robot.active = power
    }
    inverse { instant { robot.active = old!!  } }
}.name("powerIntakeCommand($power)")

fun flapCommand(robot: Robot, closed: Boolean) = instant {
    robot.flap = if(closed) Tuning.FLAP_CLOSED else Tuning.FLAP_OPEN
} + wait(Tuning.FLAP_TIME)

fun intakeCommand(robot: Robot, dist: Metre, lock: Boolean = true) = parallel(
    extendCommand(robot,dist, lock),
    powerIntakeCommand(robot, Tuning.ACTIVE_POWER),
    flapCommand(robot, true)
)

//fun detectCommand(robot: Robot) = cmd {
//    finishWhen { robot.io.distance() < Color.DIST_THRESHOLD || }
//}

fun brakeIntakeRollers(robot: Robot)
    = series(
        instant { robot.active = Tuning.ACTIVE_STOP_POWER },
        wait(Tuning.ACTIVE_STOP_TIME),
        instant { robot.active = 0.0 }
    ).name("brakeIntakeRollers")

val MIN_AUTO_INTAKE_TIME = 300.ms
var numAutoIntakes = 0

// horrible code practice lmao
private var runningAutoInputCmd: Command? = null
fun autoIntake(robot: Robot, dist: Metre): Command {
    val cmd = (deadline(
        cmd {
            var expireTime: Expression = 0.s
            init {
                expireTime = robot.io.time() + MIN_AUTO_INTAKE_TIME
                numAutoIntakes += 1
            }
            loop {
                if (robot.io.time() > expireTime && robot.slides.t.axis1 < 10.cm) {
                    status = Status.CANCELLED
                    numAutoIntakes -= 1
                }
            }
            finishWhen { robot.io.distance() < Color.DIST_THRESHOLD }
            onFinish { numAutoIntakes -= 1 }
        },
        intakeCommand(robot, dist).name("autoIntakeIntakeCommand")
    ) /*then brakeIntakeRollers(robot)*/ then powerIntakeCommand(robot,0.0) then transferCommand(robot)).name("autoIntake")

    return instant {
        runningAutoInputCmd?.status = Status.CANCELLED;
        runningAutoInputCmd = cmd
    } then cmd
}

fun retract(robot: Robot, lock: Boolean = true)
    = parallel(
        extendCommand(robot,(0).cm,lock) then instant { robot.slides.t.axis1 = (-10).cm },
        powerIntakeCommand(robot,0.0),
    ).name("retractCommand")

// TODO: maybe eject through top, needs testing.
fun eject(robot: Robot) =
    series(
        powerIntakeCommand(robot, -Tuning.ACTIVE_POWER),
        wait(200.ms),
        powerIntakeCommand(robot, 0.0)
    )

fun getSample(robot: Robot) = series(
    flapCommand(robot,true),
    powerIntakeCommand(robot, Tuning.ACTIVE_POWER),
    cmd { finishWhen { robot.io.distance() < Color.DIST_THRESHOLD } }.timeout(1.s),
    powerIntakeCommand(robot, 0.0)
)

fun wait(t: Second) = cmd {
    val curTime = ElapsedTime()
    init { curTime.reset() }
    finishWhen { curTime.seconds().s > t }
}.name("wait($t)")