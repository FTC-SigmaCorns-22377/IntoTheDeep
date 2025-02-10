package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import eu.sirotin.kotunil.base.Second
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.Expression
import eu.sirotin.kotunil.core.div
import eu.sirotin.kotunil.core.minus
import eu.sirotin.kotunil.core.plus
import eu.sirotin.kotunil.core.times
import eu.sirotin.kotunil.core.unaryMinus
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.command.Scheduler
import net.unnamedrobotics.lib.command.groups.parallel
import net.unnamedrobotics.lib.command.groups.then
import net.unnamedrobotics.lib.command.schedule
import net.unnamedrobotics.lib.gamepad.GamepadEx
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.vec2
import sigmacorns.common.Robot
import sigmacorns.common.RobotVisualizer
import sigmacorns.common.cmd.ScorePosition
import sigmacorns.common.cmd.clawCommand
import sigmacorns.common.cmd.depoCommand
import sigmacorns.common.cmd.instant
import sigmacorns.common.cmd.score
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyInputPose
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.constants.Limits
import sigmacorns.constants.Tuning
import kotlin.math.absoluteValue
import kotlin.math.sign

@TeleOp
class Teleop: SimOrHardwareOpMode() {
    lateinit var robot: Robot
    lateinit var maxSpeed: Expression
    lateinit var maxAngSpeed: Expression

    var scoringPosition: ScorePosition? = null
        set(value) {
            if(field!=value && value!=null) depoCommand(robot,value).schedule()
            field = value
        }

    fun atWallPosition() = robot.slides.t.axis2 == Tuning.specimenWallPose.lift && robot.arm.t.axis1.value.sign < 0.0

    fun clawClosed() = robot.io.claw.let { (it-Tuning.CLAW_CLOSED).absoluteValue < (it-Tuning.CLAW_OPEN).absoluteValue }

    override fun runOpMode(io: SigmaIO) {
        robot = Robot(
            io,
            DiffyOutputPose(90.degrees, 0.rad),
            DiffyOutputPose(0.m, 0.m),
            Tuning.IntakePosition.OVER
        )

        maxSpeed = robot.drivebase.motor.topSpeed(1.0) * robot.drivebase.radius
        maxAngSpeed = 0.8 * maxSpeed / (robot.drivebase.length / 2.0 + robot.drivebase.width / 2.0)

        val visualizer = RobotVisualizer(io)

        // sample sequence: extend -> transfer -> move -> score
        // specimen sequence: wall pickup -> move -> score

        // dpad l/r: toggle specimen/sample
        // dpad u/d: toggle high/low
        // x: manual transfer
        // b: pickup from wall
        // y: extend
        // a: claw

        val g1 = GamepadEx(gamepad1)
//        val g2 = GamepadEx(gamepad2)

        Scheduler.reset()

        visualizer.init()

        waitForStart()

        robot.update(0.0)

        var lastT = io.time()
        while (opModeIsActive()) {
            val t = io.time()
            val dt = (t - lastT).cast(s)
            lastT = t

            manualControls(dt)

            if(g1.dpadUp.isJustPressed) scoringPosition = when(scoringPosition) {
                ScorePosition.LOW_SPECIMEN -> ScorePosition.HIGH_SPECIMEN
                ScorePosition.LOW_BUCKET -> ScorePosition.HIGH_BUCKET
                null -> if(robot.arm.t.axis1.value.sign < 0) ScorePosition.HIGH_SPECIMEN else ScorePosition.HIGH_BUCKET
                else -> scoringPosition
            }

            if(g1.dpadDown.isJustPressed) scoringPosition = when(scoringPosition) {
                ScorePosition.HIGH_SPECIMEN -> ScorePosition.LOW_SPECIMEN
                ScorePosition.HIGH_BUCKET -> ScorePosition.LOW_BUCKET
                null -> if(robot.arm.t.axis1.value.sign < 0) ScorePosition.LOW_SPECIMEN else ScorePosition.LOW_BUCKET
                else -> scoringPosition
            }

            if(g1.dpadRight.isJustPressed) scoringPosition = when(scoringPosition) {
                ScorePosition.HIGH_SPECIMEN -> ScorePosition.HIGH_BUCKET
                ScorePosition.LOW_SPECIMEN -> ScorePosition.LOW_BUCKET
                ScorePosition.HIGH_BUCKET -> ScorePosition.HIGH_SPECIMEN
                ScorePosition.LOW_BUCKET -> ScorePosition.LOW_SPECIMEN
                null -> null
            }

            if(g1.a.isJustPressed) {
                val cmd = if(clawClosed())  {
                    val returnDepo = when(scoringPosition) {
                        ScorePosition.HIGH_SPECIMEN,ScorePosition.LOW_SPECIMEN -> instant { } //depoCommand(robot,Tuning.specimenWallPose)
                        else -> depoCommand(robot,Tuning.TRANSFER_HOVER_POSE)
                    }

                    score(robot,scoringPosition) then instant { scoringPosition=null } then returnDepo
                } else {
                    clawCommand(robot,true).let {
                        if(atWallPosition())
                            it then instant { scoringPosition = ScorePosition.LOW_SPECIMEN }
                        else
                            it
                    }
                }

                cmd.schedule()
            }

            if(g1.b.isJustPressed) parallel(
                clawCommand(robot,false),
                depoCommand(robot, Tuning.specimenWallPose),
                instant { scoringPosition = null }
            ).schedule()

            Scheduler.tick()

            if(SIM) Thread.sleep(50)

            g1.periodic()
//            g2.periodic()
            robot.update(dt.value)
            visualizer.log()
//            println(scoringPosition?.name ?: "null")
//            println("wall pos = $atWallPosition")
//            println("arm = ${robot.arm.t.axis1}")
//            println("wrist = ${robot.arm.t.axis2}")
        }
    }

    fun manualControls(dt: Second) {
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
    }
}