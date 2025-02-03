package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.ms
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.Expression
import eu.sirotin.kotunil.core.div
import eu.sirotin.kotunil.core.minus
import eu.sirotin.kotunil.core.plus
import eu.sirotin.kotunil.core.times
import eu.sirotin.kotunil.core.unaryMinus
import eu.sirotin.kotunil.derived.rad
import kotlinx.coroutines.runBlocking
import net.unnamedrobotics.lib.command.Command
import net.unnamedrobotics.lib.command.Scheduler
import net.unnamedrobotics.lib.command.Status
import net.unnamedrobotics.lib.command.cmd
import net.unnamedrobotics.lib.command.groups.then
import net.unnamedrobotics.lib.command.schedule
import net.unnamedrobotics.lib.gamepad.GamepadEx
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.map
import net.unnamedrobotics.lib.math2.vec2
import net.unnamedrobotics.lib.rerun.rerun
import sigmacorns.common.Robot
import sigmacorns.common.RobotVisualizer
import sigmacorns.common.cmd.armCommand
import sigmacorns.common.cmd.autoIntake
import sigmacorns.common.cmd.depoCommand
import sigmacorns.common.cmd.liftCommand
import sigmacorns.common.cmd.retract
import sigmacorns.common.cmd.transferCommand
import sigmacorns.common.cmd.wait
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyInputPose
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.common.kinematics.LiftPose
import sigmacorns.constants.Limits
import sigmacorns.constants.Tuning

@TeleOp
class Teleop: SimOrHardwareOpMode() {
    enum class ScorePositions(val x: LiftPose) {
        HIGH_SPECIMEN(Tuning.specimenHighPose),
        LOW_SPECIMEN(Tuning.specimenLowPose),
        HIGH_BUCKET(Tuning.bucketHighPose),
        LOW_BUCKET(Tuning.bucketLowPose)
    }

    data class CommandHolder(var cmd: Command)

    lateinit var robot: Robot

    var wasAPressed = false
    var wasBPressed = false
    var wasYPressed = false
    var wasXPressed = false
    var wasRBumperPressed = 0
    val rBumperThresh = 5
    var wasLBumperPressed = false

    var dt = 0.s
    var clawOpen = true

    var intaking = false
    var intakingDist: Expression = 0.m

    private fun openClaw() {
        clawOpen = true
        intaking = false
        robot.claw.updatePort(Tuning.CLAW_OPEN)
    }

    private fun runClawOpen() {
//        if (gamepad1.y && !wasYPressed) {
//            intaking = true
//            intakingDist = 0.35.m
//
//            (autoIntake(
//                robot,
//                intakingDist.cast(m)
//            ) then cmd { instant { closeClaw() } }).schedule()
//        }

        if (gamepad1.left_bumper)
            liftCommand(robot, 0.m).schedule()

        if (gamepad1.b && !wasBPressed)
            depoCommand(robot, Tuning.specimenWallPose).schedule()

        if (gamepad1.x && !wasXPressed)
            (transferCommand(robot) then cmd { instant { closeClaw() } }).schedule()

        if (intaking) {
            if (gamepad1.dpad_up) intakingDist += dt * 0.2.m / s
            if (gamepad1.dpad_down) intakingDist -= dt * 0.2.m / s

            robot.slides.t.let {
                DiffyOutputPose(intakingDist, it.axis2)
            }

//            if (gamepad1.y && !wasYPressed) {
//                retract(robot).schedule()
//                intaking = false
//            }
        }

        if (gamepad2.right_bumper && wasRBumperPressed > rBumperThresh) robot.intake.t = when (robot.intake.t) {
            Robot.IntakePositions.OVER -> Robot.IntakePositions.INTER
            Robot.IntakePositions.INTER -> Robot.IntakePositions.OVER
            Robot.IntakePositions.BACK -> TODO()
        }

        if (gamepad1.a && !wasAPressed) {
            wasAPressed = true
            closeClaw()
        }
    }

    private var dst: ScorePositions? = null
    private fun closeClaw() {
        dst = null
        clawOpen = false
        robot.intake.follow(Robot.IntakePositions.OVER).schedule()
        robot.claw.updatePort(Tuning.CLAW_CLOSED)
    }

    private fun runClawClose() {
        robot.slides.t = robot.slides.t.let {
            DiffyOutputPose(
                it.axis1,
                Limits.LIFT.apply(
                    (it.axis2 + 0.2.m / s * dt * (gamepad1.dpad_up.toInt() - gamepad1.dpad_down.toInt())).cast(
                        m
                    )
                )
            )
        }

        if (gamepad1.a && !wasAPressed) {
            val opencmd = cmd { instant { openClaw() } }
            val c = when (dst) {
                ScorePositions.HIGH_SPECIMEN, ScorePositions.LOW_SPECIMEN ->
                    liftCommand(
                        robot,
                        (dst!!.x.lift + Tuning.specimentScoreOffset).cast(m)
                    ) then opencmd

                ScorePositions.HIGH_BUCKET, ScorePositions.LOW_BUCKET ->
                    opencmd then wait(300.ms) then armCommand(
                        robot,
                        robot.arm.t.axis1.map { -it }.cast(rad),
                        robot.arm.t.axis2.map { -it }.cast(rad)
                    )

                null -> opencmd
            }
            c.schedule()
        }

        if (gamepad1.x && !wasXPressed) (
                cmd {
                    instant {
                        clawOpen = true
                    }
                } then transferCommand(robot) then cmd { instant { closeClaw() } }
                ).schedule()

        if (gamepad1.right_bumper && wasRBumperPressed > rBumperThresh) {
            dst = ScorePositions.HIGH_BUCKET
            depoCommand(robot, dst!!.x).schedule()
        }

        if (gamepad1.left_bumper && !wasLBumperPressed) {
            dst = ScorePositions.LOW_BUCKET
            depoCommand(robot, dst!!.x).schedule()
        }

        if (gamepad1.b && !wasBPressed) {
            dst = ScorePositions.LOW_SPECIMEN
            depoCommand(robot, dst!!.x).schedule()
        }

        if (gamepad1.y && !wasYPressed) {
            dst = ScorePositions.HIGH_SPECIMEN
            depoCommand(robot, dst!!.x).schedule()
        }

//        rerun(io?.rerunConnection!!) {
//             scalar("sjgsgfojn",)
//        }
    }

    override fun runOpMode(io: SigmaIO) {
        robot = Robot(
            io,
            DiffyOutputPose(90.degrees, 0.rad),
            DiffyOutputPose(0.m, 0.m),
            Robot.IntakePositions.OVER
        )

//        val visualizer = RobotVisualizer(io)

        val g1 = GamepadEx(gamepad1)
        val g2 = GamepadEx(gamepad2)

        val maxSpeed = robot.drivebase.motor.topSpeed(1.0) * robot.drivebase.radius
        val maxAngSpeed = 0.8 * maxSpeed / (robot.drivebase.length / 2.0 + robot.drivebase.width / 2.0)


        Scheduler.reset()

//        visualizer.init()

        waitForStart()

        robot.update(0.0)

        openClaw()

        var lastT = io.time()
        while (opModeIsActive()) {
            val t = io.time()
            dt = (t - lastT).cast(s)
            lastT = t

            val v = vec2(-gamepad1.left_stick_y, gamepad1.left_stick_x)
            robot.mecanum.t = Transform2D(v * maxSpeed, -maxAngSpeed * gamepad1.right_stick_x)

            val activePower =
                gamepad1.left_trigger - gamepad1.right_trigger + gamepad2.left_trigger - gamepad2.right_trigger
            robot.active.updatePort(activePower * Tuning.ACTIVE_POWER)

            // MANUAL SLIDES/ARM
            var slidesTarget = robot.slides.t
            slidesTarget = DiffyOutputPose(
                slidesTarget.axis1 - gamepad2.left_stick_y * dt * 20.cm / s,
                slidesTarget.axis2 - gamepad2.right_stick_y * dt * 20.cm / s,
            )

            slidesTarget.axis1 = Limits.EXTENSION.apply(slidesTarget.axis1.cast(m))
            slidesTarget.axis2 = Limits.LIFT.apply(slidesTarget.axis2.cast(m))

            val armPower = gamepad2.dpad_up.toInt() - gamepad2.dpad_down.toInt()
            val wristPower = gamepad2.dpad_right.toInt() - gamepad2.dpad_left.toInt()

            robot.arm.t = robot.arm.kinematics.underInverse(robot.arm.t.let {
                DiffyOutputPose(
                    it.axis1 - armPower * dt * 0.5.rad / s,
                    it.axis2 - wristPower * dt * 0.5.rad / s
                )
            }) {
                DiffyInputPose(
                    Limits.ARM_SERVO_1.apply(it.axis1.cast(rad)),
                    Limits.ARM_SERVO_2.apply(it.axis2.cast(rad))
                )
            }

            robot.slides.t = slidesTarget

            // END MANUAL POWER

            g1.periodic()
            g2.periodic()

            if (clawOpen) runClawOpen() else runClawClose()

            Scheduler.tick()
            wasAPressed = gamepad1.a
            wasBPressed = gamepad1.b
            wasXPressed = gamepad1.x
            wasYPressed = gamepad1.y
            wasLBumperPressed = gamepad1.left_bumper
            wasRBumperPressed = if(gamepad1.right_bumper) 0 else wasRBumperPressed+1
//            wasRBumperPressed += gamepad1.right_bumper

            robot.update(dt.value)
//            visualizer.log()

            // AUTOMATED: instant extend, start intaking
            // instant transfer
            // presets
        }
    }
}

    /*
    this would be great as a behavior tree.

    Claw open
        -> extend -> transfer -> Claw closed
                  -> wall
        -> wall -> Claw closed

    Claw closed (extend in)
        -> low specimen
        -> high specimen
        -> low sample
        -> high sample
        -> drop -> Claw open
        -> transfer

     */

