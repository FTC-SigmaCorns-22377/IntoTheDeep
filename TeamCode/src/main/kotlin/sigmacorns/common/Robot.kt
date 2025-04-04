package sigmacorns.common

import dev.nullrobotics.sample.SwerveSample
import dev.nullrobotics.trajectory.Trajectory
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.core.Expression
import eu.sirotin.kotunil.derived.Volt
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.command.Command
import net.unnamedrobotics.lib.command.schedule
import net.unnamedrobotics.lib.control.controller.transfer
import net.unnamedrobotics.lib.math2.Bounds
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.xy
import net.unnamedrobotics.lib.physics.MecanumDrivebase
import net.unnamedrobotics.lib.physics.goBildaMotorConstants
import sigmacorns.common.cmd.CommandSlot
import sigmacorns.common.control.Actuator
import sigmacorns.common.control.ArmControlLoop
import sigmacorns.common.control.ChoreoController
import sigmacorns.common.control.ControlLoop
import sigmacorns.common.control.MecanumController
import sigmacorns.common.control.PIDDiffyController
import sigmacorns.common.control.choreoControllerLoop
import sigmacorns.common.control.controlLoop
import sigmacorns.common.control.slidesControlLoop
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyInputPose
import sigmacorns.common.kinematics.DiffyKinematics
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.common.kinematics.IntakeAngleKinematics
import sigmacorns.constants.IntakePosition
import sigmacorns.constants.Limits
import sigmacorns.constants.Physical
import sigmacorns.constants.Tuning
import sigmacorns.constants.*
import java.util.Optional

class Robot(
    val io: SigmaIO,
    val initArmPose: DiffyOutputPose = DiffyOutputPose(0.rad, 0.rad),
    initSlidesPose: DiffyOutputPose = DiffyOutputPose(0.m, 0.m),
    intakePos: IntakePosition = IntakePosition.OVER,
    initPos: Transform2D = Transform2D(0.m, 0.m, 0.rad)
) {
    val drivebase: MecanumDrivebase = MecanumDrivebase(
        Physical.WHEEL_RADIUS,
        Physical.DRIVEBASE_SIZE.y.cast(m),
        Physical.DRIVEBASE_SIZE.x.cast(m),
        Physical.WEIGHT,
        Physical.WHEEL_INERTIA,
        goBildaMotorConstants(Physical.DRIVE_RATIO)
    )

    val slidesController = PIDDiffyController(
        DiffyKinematics(Physical.EXTEND_M_PER_TICK, Physical.LIFT_M_PER_TICK),
        Tuning.EXTENSION_PID,
        Tuning.LIFT_PID,
        Limits.EXTENSION as Bounds<Expression>,
        Limits.LIFT as Bounds<Expression>,
        Limits.EXTENSION_SAFE_THRESH,
        Limits.LIFT_SAFE_THRESH,
        Limits.EXTENSION_SAFE_POWER,
        Limits.LIFT_SAFE_POWER,
        Limits.SLIDE_MOTOR_MAX
    )

    val mecanum = MecanumController(drivebase, io)
    private val choreoController =
        ChoreoController(Tuning.choreoPosPID, Tuning.choreoAngPID, Physical.DRIVEBASE_SIZE.xy, 0)
    val choreo = choreoControllerLoop(choreoController, this)
    val trajLogger = choreoController.trajectoryLogger
    val slides: ControlLoop<DiffyInputPose, List<Volt>, DiffyOutputPose> =
        slidesControlLoop(slidesController, io, initSlidesPose)
    val arm =
        with(io) { ArmControlLoop(Actuator { armL = it }, Actuator { armR = it }, initArmPose, io) }

    val intake = controlLoop(
        transfer { d, x: Unit, t: IntakePosition ->
            IntakeAngleKinematics.inverse(t.x)
        }, "intake", io, {}, {
            io.intakeL = Limits.INTAKE_SERVO_1.toServoPos()(it); io.intakeR =
            Limits.INTAKE_SERVO_2.toServoPos()(it)
        }).also { it.t = intakePos }

    fun resetSlots() {
        extendCommandSlot.curCmd = null
        liftCommandSlot.curCmd = null
        armCommandSlot.curCmd = null
        intakeCommandSlot.curCmd = null
    }

    val claw: Actuator<Double> = with(io) { Actuator { claw = it } }
    val active: Actuator<Double> =
        with(io) { Actuator({ Exception().printStackTrace() }) { intake = it } }
    val flap: Actuator<Double> = with(io) { Actuator { flap = it } }

    val extendCommandSlot = CommandSlot()
    val liftCommandSlot = CommandSlot()
    val armCommandSlot = CommandSlot()
    val intakeCommandSlot = CommandSlot()

    init {
        io.setPinPos(initPos)
        extendCommandSlot.schedule()
        liftCommandSlot.schedule()
        armCommandSlot.schedule()
        intakeCommandSlot.schedule()
    }

    fun update(dt: Double) {
        mecanum.tickControlNode(dt)
        slides.tickControlNode(dt)
        arm.tickControlNode(dt)
        intake.tickControlNode(dt)
        active.node.tickControlNode(dt)
        claw.node.tickControlNode(dt)
        flap.node.tickControlNode(dt)
        io.updateColorDist()
        io.updatePinpoint()
    }

    fun followPath(traj: Trajectory<SwerveSample>, fast: Boolean = false): Command {
        val cmd = choreo.follow(Optional.of(traj))
        if (fast) {
            choreoController.t = 0.15
            choreoController.tPreset = true
        } else {
            choreoController.t = 0.0
            choreoController.tPreset = false
        }
        return cmd
    }
}