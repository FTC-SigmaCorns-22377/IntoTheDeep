package sigmacorns.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nullrobotics.Choreo
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.mm
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.command.Scheduler
import net.unnamedrobotics.lib.command.groups.then
import net.unnamedrobotics.lib.command.schedule
import net.unnamedrobotics.lib.command.wait
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.vec2
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import sigmacorns.common.AutoSamples
import sigmacorns.common.Robot
import sigmacorns.common.cmd.follow
import sigmacorns.common.cmd.hover
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.subsystems.arm.ScoringPose
import sigmacorns.common.subsystems.swerve.swerveControlLoop
import sigmacorns.opmode.SimOrHardwareOpMode
import kotlin.time.Duration
import kotlin.time.Duration.Companion.seconds

@Autonomous
class BlueBucketAuto: SimOrHardwareOpMode() {
    val firstTraj = (Choreo::loadTrajectory)("preload").get()
    override val initialScoringPose = firstTraj.initialPose.let { ScoringPose(
        vec2(it.getX(DistanceUnit.METER).m,it.getY(DistanceUnit.METER).m),
        it.getHeading(AngleUnit.RADIANS).rad,
        406.4.mm,
        90.degrees,
        0.rad,0.rad
    ) }

    override fun runOpMode(io: SigmaIO) {
        val robot = Robot(io)

        robot.addLoop(robot.armControlLoop)

        robot.addLoop(robot.choreoControlLoop)
        robot.choreoControlLoop.enabled = false

        robot.addLoop(robot.swerveControlLoop)

        robot.addLoop(robot.swerveHeadingControlLoop)
        robot.swerveHeadingControlLoop.enabled = false

        robot.addLoop(robot.swerveLogPosControlLoop)
        robot.addLoop(robot.swerveLogVelControlLoop)

        val cmd = (robot.follow("preload") then
            wait(3.seconds) then robot.hover("Pickup 1", AutoSamples.BLUE_BUCKET_SAMPLE_1,0.rad) then
            wait(3.seconds) then robot.follow("Score 1"))
        cmd.schedule()

        waitForStart()

        robot.launchIOLoop()

        robot.inputLoop {
            Scheduler.tick()
        }
    }

}