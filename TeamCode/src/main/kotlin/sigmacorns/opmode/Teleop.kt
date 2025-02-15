package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import eu.sirotin.kotunil.base.Second
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.mm
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
import net.unnamedrobotics.lib.command.groups.plus
import net.unnamedrobotics.lib.command.groups.then
import net.unnamedrobotics.lib.command.schedule
import net.unnamedrobotics.lib.gamepad.GamepadEx
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.Twist2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.vec2
import sigmacorns.common.Robot
import sigmacorns.common.RobotVisualizer
import sigmacorns.common.cmd.ScorePosition
import sigmacorns.common.cmd.autoIntake
import sigmacorns.common.cmd.clawCommand
import sigmacorns.common.cmd.depoCommand
import sigmacorns.common.cmd.instant
import sigmacorns.common.cmd.numAutoIntakes
import sigmacorns.common.cmd.score
import sigmacorns.common.cmd.transferCommand
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyInputPose
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.constants.Limits
import sigmacorns.constants.Tuning
import kotlin.math.absoluteValue
import kotlin.math.sign

@TeleOp(name = "_TELEOP_")
class Teleop: SimOrHardwareOpMode() {
    lateinit var robot: Robot
    lateinit var maxSpeed: Expression
    lateinit var maxAngSpeed: Expression

    var scoringPosition: ScorePosition? = null
        set(value) {
            if(field!=value && value!=null) depoCommand(robot,value).schedule()
            field = value
        }

    var tiltPosition = Tuning.TiltPositions.STRAIGHT
        set(value) {
            io!!.tilt1 = value.x
            io!!.tilt2 = value.x
            field = value
        }

    fun atWallPosition() = robot.slides.t.axis2 == Tuning.specimenWallPose.lift && robot.arm.t.axis1.value.sign < 0.0

    fun clawClosed() = robot.io.claw.let { (it-Tuning.CLAW_CLOSED).absoluteValue < (it-Tuning.CLAW_OPEN).absoluteValue }

    lateinit var g1: GamepadEx
    lateinit var g2: GamepadEx

    override fun runOpMode(io: SigmaIO) {
        g1 = GamepadEx(gamepad1)
        g2 = GamepadEx(gamepad2)

        robot = Robot(
            io,
            DiffyOutputPose(90.degrees, 0.rad),
            DiffyOutputPose(0.m, 0.m),
            Tuning.IntakePosition.OVER
        )

        val visualizer = if(SIM) RobotVisualizer(io) else null

        maxSpeed = robot.drivebase.motor.topSpeed(1.0) * robot.drivebase.radius
        maxAngSpeed = 0.8 * maxSpeed / (robot.drivebase.length / 2.0 + robot.drivebase.width / 2.0)


        // sample sequence: extend -> transfer -> move -> score
        // specimen sequence: wall pickup -> move -> score

        // dpad l/r: toggle specimen/sample
        // dpad u/d: toggle high/low
        // x: manual transfer
        // b: pickup from wall
        // y: extend
        // a: claw


        visualizer?.init()
        Scheduler.reset()

        waitForStart()

        var lastT = io.time()
        while (opModeInInit()) {
            val t = io.time()
            val dt = (t - lastT).cast(s)
            lastT = t

            robot.update(0.0)
        }

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
                    score(robot,scoringPosition) + instant { scoringPosition=null }
                } else {
                    clawCommand(robot,true).let {
                        if(atWallPosition())
                            it then instant { scoringPosition = ScorePosition.HIGH_SPECIMEN }
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

            if(g1.x.isJustPressed) {
                // x is also used to retract the lift slides when they are up and no sample is detected in the intakez
                transferCommand(robot).schedule()
            }

            if(g1.y.isJustPressed) autoIntake(robot,300.mm).schedule()

            println("RUNNING COMMANDS")
            for(cmd in Scheduler.cmds) {
                print("${cmd.name}(${cmd.status}), ")
            }
            println("--------------")
            println("numAutoIntakes = $numAutoIntakes")

            Scheduler.tick()

            if(SIM) Thread.sleep(50)

//            telemetry.addData("distance",io.distance())
//            telemetry.update()

            g1.periodic()
            g2.periodic()
            robot.update(dt.value)
            visualizer?.log()
        }
    }

    private var wasManuallyControllingActive = false

    fun manualControls(dt: Second) {
        val v = vec2(-gamepad1.left_stick_y, -gamepad1.left_stick_x)
        robot.mecanum.t = Twist2D(v * maxSpeed, -maxAngSpeed * gamepad1.right_stick_x).exp()

        val activePower =
            gamepad1.left_trigger - gamepad1.right_trigger + gamepad2.left_trigger - gamepad2.right_trigger
        val controllingActive = activePower.absoluteValue>0.05
        if(controllingActive) robot.active.updatePort(activePower * Tuning.ACTIVE_POWER)
        if(!controllingActive && wasManuallyControllingActive) robot.active.updatePort(0.0)
        wasManuallyControllingActive = controllingActive

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

        // tilt
        if(g2.y.isJustPressed) {
            tiltPosition = tiltPosition.next()
        }
    }
}