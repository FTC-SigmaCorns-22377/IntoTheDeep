package sigmacorns.common

import dev.nullrobotics.sample.SwerveSample
import dev.nullrobotics.trajectory.Trajectory
import eu.sirotin.kotunil.base.A
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.milliampere
import eu.sirotin.kotunil.base.ms
import eu.sirotin.kotunil.core.Expression
import eu.sirotin.kotunil.core.minus
import eu.sirotin.kotunil.derived.Volt
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.command.Command
import net.unnamedrobotics.lib.command.schedule
import net.unnamedrobotics.lib.math2.Bounds
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.tick
import net.unnamedrobotics.lib.math2.xy
import net.unnamedrobotics.lib.physics.MecanumDrivebase
import net.unnamedrobotics.lib.physics.goBildaMotorConstants
import sigmacorns.common.cmd.CommandSlot
import sigmacorns.common.control.ArmControlLoop
import sigmacorns.common.control.ChoreoController
import sigmacorns.common.control.ControlLoop
import sigmacorns.common.control.MecanumControlLoop
import sigmacorns.common.control.PIDDiffyController
import sigmacorns.common.control.choreoControllerLoop
import sigmacorns.common.control.slidesControlLoop
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyInputPose
import sigmacorns.common.kinematics.DiffyKinematics
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.constants.Color
import sigmacorns.constants.Limits
import sigmacorns.constants.Physical
import sigmacorns.constants.SampleColors
import sigmacorns.constants.Tuning
import java.util.Optional

class Robot(
    val io: SigmaIO,
    val initArmPose: DiffyOutputPose = DiffyOutputPose(0.rad, 0.rad),
    initSlidesPose: DiffyOutputPose = DiffyOutputPose(0.m, 0.m),
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

    val mecanum = MecanumControlLoop(drivebase, io)
    val choreoController =
        ChoreoController(Tuning.choreoPosPID, Tuning.choreoAngPID, Physical.DRIVEBASE_SIZE.xy, 0)
    val choreo = choreoControllerLoop(choreoController, this)
    val slides: ControlLoop<DiffyInputPose, List<Volt>, DiffyOutputPose> =
        slidesControlLoop(slidesController, io, initSlidesPose)
    val arm = ArmControlLoop({ io.armL = it }, { io.armR = it }, { io.wrist = it }, initArmPose, io)

    var pto = false

    val trajLogger = choreoController.trajectoryLogger

    fun resetSlots() {
        extendCommandSlot.curCmd = null
        liftCommandSlot.curCmd = null
        armCommandSlot.curCmd = null
        intakeCommandSlot.curCmd = null
    }

    var claw: Double
        set(value) {
            io.claw = value
        }
        get() = io.claw
    var active: Double
        set(value) {
            io.intake = value
        }
        get() = io.intake
    var flap: Double
        set(value) {
            io.flap = value
        }
        get() = io.flap
    var push: Double
        set(value) {
            io.push = value
        }
        get() = io.push

    val extendCommandSlot = CommandSlot()
    val liftCommandSlot = CommandSlot()
    val armCommandSlot = CommandSlot()
    val intakeCommandSlot = CommandSlot()

    init {
        io.setPinPos(initPos)
        io.resetSlideMotors(0.tick,0.tick)
        extendCommandSlot.schedule()
        liftCommandSlot.schedule()
        armCommandSlot.schedule()
        intakeCommandSlot.schedule()
    }

    var lastIntakeStallTime = 0.ms
    var wasAntiJamming = false

    fun update(dt: Double) {
        io.pto1 = if(pto) Tuning.PTO_L_ACTIVE else Tuning.PTO_L_INACTIVE
        io.pto2 = if(pto) Tuning.PTO_R_ACTIVE else Tuning.PTO_R_INACTIVE

        if(io.intakeLimitTriggered() && io.liftLimitTriggered()) {
            io.resetSlideMotors(0.tick,0.tick)
        }

        if(io.intakeCurrent() > 6.5.A) {
            lastIntakeStallTime = io.time()
        }

        if(io.time()-lastIntakeStallTime < 500.ms) {
            io.intake = -Tuning.ACTIVE_POWER
            wasAntiJamming = true
        } else if (wasAntiJamming) {
            wasAntiJamming = false
            io.intake = Tuning.ACTIVE_POWER
        }


        //else if(io.intakeLimitTriggered()) {
//            val new = slidesController.kinematics.under(slidesController.position) {
//                DiffyOutputPose(0.m,it.axis2)
//            }
//            io.resetSlideMotors(new.axis1.cast(tick),new.axis2.cast(tick))
//        } else if(io.liftLimitTriggered()) {
//            val new = slidesController.kinematics.under(slidesController.position) {
//                DiffyOutputPose(it.axis1,0.m)
//            }
//            io.resetSlideMotors(new.axis1.cast(tick),new.axis2.cast(tick))
//        }

        mecanum.tickControlNode(dt)
        slides.tickControlNode(dt)
        arm.tickControlNode(dt)
        io.updateSensors()
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

    fun color(): SampleColors? = Color.color(io.distance(),io.red(),io.green(),io.blue())
}