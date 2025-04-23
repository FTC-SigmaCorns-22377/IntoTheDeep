package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nullrobotics.Choreo
import dev.nullrobotics.sample.SwerveSample
import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.mm
import eu.sirotin.kotunil.base.ms
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.div
import eu.sirotin.kotunil.core.minus
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.command.Command
import net.unnamedrobotics.lib.command.Scheduler
import net.unnamedrobotics.lib.command.cmd
import net.unnamedrobotics.lib.command.groups.parallel
import net.unnamedrobotics.lib.command.groups.plus
import net.unnamedrobotics.lib.command.groups.series
import net.unnamedrobotics.lib.command.groups.then
import net.unnamedrobotics.lib.command.schedule
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.inches
import net.unnamedrobotics.lib.math2.vec3
import net.unnamedrobotics.lib.rerun.archetypes.Boxes3D
import net.unnamedrobotics.lib.rerun.rerun
import org.joml.Quaterniond
import sigmacorns.common.Robot
import sigmacorns.common.cmd.ScorePosition
import sigmacorns.common.cmd.autoIntake
import sigmacorns.common.cmd.choreoCommand
import sigmacorns.common.cmd.clawCommand
import sigmacorns.common.cmd.depoCommand
import sigmacorns.common.cmd.extendCommand
import sigmacorns.common.cmd.getSample
import sigmacorns.common.cmd.intakeCommand
import sigmacorns.common.cmd.liftCommand
import sigmacorns.common.cmd.reset
import sigmacorns.common.cmd.score
import sigmacorns.common.cmd.wait
import sigmacorns.common.control.ChoreoController
import sigmacorns.common.control.TrajectoryLogger
import sigmacorns.common.control.toTransform2d
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.common.kinematics.LiftPose
import sigmacorns.constants.Physical
import sigmacorns.constants.Tuning
import sigmacorns.opmode.SimOrHardwareOpMode

@Autonomous
class SampleAuto: SimOrHardwareOpMode() {
    override fun runOpMode(io: SigmaIO) {
        val robot = Robot(
            io,
            DiffyOutputPose(90.degrees, 90.degrees),
            DiffyOutputPose(0.m, 0.m),
            initPos = startPosFromTraj("Prepreload")
        )

//        Scheduler.log = true
        Scheduler.reset()

        var lastT = io.time()
        while (opModeInInit()) {
            val t = io.time()
            val dt = (t - lastT).cast(s)
            lastT = t

            robot.claw=Tuning.CLAW_CLOSED

            robot.update(dt.value)
        }

        waitForStart()

        robot.update(0.0)
        sampleAuto(robot).schedule()

        while (opModeIsActive()) {
            val t = io.time()
            val dt = (t - lastT).cast(s)
            lastT = t

            println("RUNNING COMMANDS")
            for(cmd in Scheduler.cmds) {
                print("${cmd.name}(${cmd.status}), ")
            }
            println("--------------")

            Scheduler.tick()
            robot.choreo.tickControlNode(dt.value)
            robot.update(dt.value)

            rerun(robot.io.rerunConnection) {
                if(robot.choreo.t.isPresent) {
                    robot.trajLogger.logTraj(robot.choreo.t.get(), TrajectoryLogger.Mode.POINTS)
                    robot.trajLogger.highlightSample("bot",robot.choreoController.sample)
                    val s = robot.choreoController.position.pos
                    log("pos") { Boxes3D(
                        halfSizes = listOf( vec3(
                            Physical.DRIVEBASE_SIZE.x/2.0,
                            Physical.DRIVEBASE_SIZE.y/2.0,0.m) ),
                        centers = listOf(vec3(s.x,s.y,0.m)),
                        rotations = listOf(Quaterniond().setAngleAxis(s.angle.value,0.0,0.0,1.0))
                    ) }
                }
            }
        }
    }
}

fun sampleAuto(robot: Robot) =
    series(
        choreoCommand(robot, "Prepreload") + depoCommand(robot, Tuning.bucketHighPose),
        choreoCommand(robot,"Preload"),
        score(robot, ScorePosition.HIGH_BUCKET),
        parallel(
            choreoCommand(robot, "Pickup_1"),
            depoCommand(robot, Tuning.TRANSFER_POSE.let { LiftPose(500.mm,it.arm,it.wrist) }),
            ),
        liftCommand(robot,0.m) /*+ autoIntake(robot,14.inches)*/,
//        choreoCommand(robot, "Score_1") + depoCommand(robot, Tuning.bucketHighPose),
//        score(robot, ScorePosition.HIGH_BUCKET),
//        reset(robot),
//        choreoCommand(robot, "Pickup_2") + autoIntake(robot, 20.5.inches),
//        choreoCommand(robot, "Score_2") + depoCommand(robot, Tuning.bucketHighPose),
//        score(robot, ScorePosition.HIGH_BUCKET),
//        reset(robot),
//        choreoCommand(robot, "Pickup_3") + autoIntake(robot, 20.5.inches),
//        choreoCommand(robot, "Score_3") + depoCommand(robot, Tuning.bucketHighPose),
//        score(robot, ScorePosition.HIGH_BUCKET),
//        reset(robot),
//        choreoCommand(robot, "Ascent")
    )
