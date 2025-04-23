package sigmacorns.common.cmd

import com.qualcomm.robotcore.util.ElapsedTime
import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.Second
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.ms
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.*
import net.unnamedrobotics.lib.command.Command
import net.unnamedrobotics.lib.command.Scheduler
import net.unnamedrobotics.lib.command.Status
import net.unnamedrobotics.lib.command.cmd
import net.unnamedrobotics.lib.command.forever
import net.unnamedrobotics.lib.command.groups.deadline
import net.unnamedrobotics.lib.command.groups.parallel
import net.unnamedrobotics.lib.command.groups.plus
import net.unnamedrobotics.lib.command.groups.race
import net.unnamedrobotics.lib.command.groups.series
import net.unnamedrobotics.lib.command.groups.then
import net.unnamedrobotics.lib.command.schedule
import sigmacorns.common.Robot
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.constants.Color
import sigmacorns.constants.FlapPosition
import sigmacorns.constants.SampleColors
import sigmacorns.constants.Tuning
import kotlin.reflect.jvm.internal.impl.utils.FunctionsKt

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

fun flapCommand(robot: Robot, closed: FlapPosition) = instant {
    robot.flap = closed.x
} + wait(Tuning.FLAP_TIME)

fun intakeCommand(robot: Robot, dist: Metre, lock: Boolean = true) = parallel(
    extendCommand(robot,dist, lock),
    powerIntakeCommand(robot, Tuning.ACTIVE_POWER),
    flapCommand(robot, FlapPosition.CLOSED)
)

fun brakeIntakeRollers(robot: Robot)
    = series(
        instant { robot.active = Tuning.ACTIVE_STOP_POWER },
        wait(Tuning.ACTIVE_STOP_TIME),
        instant { robot.active = 0.0 }
    ).name("brakeIntakeRollers")

val MIN_AUTO_INTAKE_TIME = 300.ms
var numAutoIntakes = 0

// horrible code practice lmao
var runningAutoInputCmd: Command? = null
fun autoIntake(robot: Robot, dist: Metre, maxTime: Second? = null, colors: Set<SampleColors> = setOf(SampleColors.YELLOW,SampleColors.RED,SampleColors.BLUE)): Command {
    val cmd = parallel(
        cmd {
                finishWhen { robot.slidesController.kinematics.forward(robot.slides.x).axis1<20.cm }
        } then flapCommand(robot,FlapPosition.CLOSED),
        deadline(
            cmd {
                var lastEjectTime: Second? = null
                run {
                    if(robot.color()!=null && robot.color() !in colors) {
                        robot.flap = FlapPosition.EJECT.x
                        lastEjectTime = robot.io.time()
                    } else if(lastEjectTime==null ||  robot.io.time()-lastEjectTime!! > 500.ms) {
                        robot.flap = FlapPosition.CLOSED.x
                    }

                    return@run colors.contains(robot.color()).also { if(it) robot.flap = FlapPosition.CLOSED.x }
                }
            }.let { if(maxTime!=null) it.timeout(maxTime) else it },
        intakeCommand(robot, dist).name("autoIntakeIntakeCommand") then forever { false }
        ) then (
                powerIntakeCommand(robot,0.0)
         + instant { transferCommand(robot).schedule() })).name("autoIntake")

    return instant {
        runningTransferCommand?.status = Status.CANCELLED
        runningTransferCommand = null
        runningAutoInputCmd?.status = Status.CANCELLED
        runningAutoInputCmd = cmd
    } then cmd
}

fun retract(robot: Robot, lock: Boolean = true)
    = parallel(
        race(
            cmd {
                finishWhen { robot.slidesController.kinematics.forward(robot.slides.x).axis1 < 5.cm }
                },extendCommand(robot,(-20).cm,lock)
        ).timeout(2.s),
        powerIntakeCommand(robot,0.0),
    ).name("retractCommand")

// TODO: maybe eject through top, needs testing.
fun eject(robot: Robot) =
    series(
        powerIntakeCommand(robot, -Tuning.ACTIVE_POWER),
        wait(200.ms),
        powerIntakeCommand(robot, 0.0)
    )

fun resetIntake(robot: Robot) = cmd {
    init { robot.slidesController.bypassAxis1 = Tuning.SLIDE_RESET_POWER }
    finishWhen { robot.io.intakeLimitTriggered() }
}.timeout(3.s) then instant {
    robot.slidesController.bypassAxis1=null
}

fun getSample(robot: Robot) = series(
    flapCommand(robot,FlapPosition.CLOSED),
    powerIntakeCommand(robot, Tuning.ACTIVE_POWER),
    cmd { finishWhen { robot.color()!=null } }.timeout(1.s),
    powerIntakeCommand(robot, 0.0)
)

fun wait(t: Second) = cmd {
    val curTime = ElapsedTime()
    init { curTime.reset() }
    finishWhen { curTime.seconds().s > t }
}.name("wait($t)")