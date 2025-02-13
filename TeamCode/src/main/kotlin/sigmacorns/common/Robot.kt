package sigmacorns.common

import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.command.schedule
import net.unnamedrobotics.lib.control.controller.transfer
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.xy
import net.unnamedrobotics.lib.physics.MecanumDrivebase
import net.unnamedrobotics.lib.physics.goBildaMotorConstants
import sigmacorns.common.cmd.CommandSlot
import sigmacorns.common.control.Actuator
import sigmacorns.common.control.ArmControlLoop
import sigmacorns.common.control.ChoreoController
import sigmacorns.common.control.MecanumController
import sigmacorns.common.control.ServoController
import sigmacorns.common.control.choreoControllerLoop
import sigmacorns.common.control.slidesControlLoop
import sigmacorns.common.control.toControlLoop
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.DiffyOutputPose
import sigmacorns.common.kinematics.IntakeAngleKinematics
import sigmacorns.constants.Limits
import sigmacorns.constants.Physical
import sigmacorns.constants.Tuning
import sigmacorns.constants.toServoPos

class Robot(
    val io: SigmaIO,
    val initArmPose: DiffyOutputPose = DiffyOutputPose(0.rad,0.rad),
    val initSlidesPose: DiffyOutputPose = DiffyOutputPose(0.m,0.m),
    intakePos: Tuning.IntakePosition = Tuning.IntakePosition.OVER,
    initPos: Transform2D = Transform2D(0.m,0.m,0.rad)
) {
    val drivebase: MecanumDrivebase = MecanumDrivebase(
        Physical.WHEEL_RADIUS,
        Physical.DRIVEBASE_SIZE.y.cast(m),
        Physical.DRIVEBASE_SIZE.x.cast(m),
        Physical.WEIGHT,
        Physical.WHEEL_INERTIA,
        goBildaMotorConstants(Physical.DRIVE_RATIO)
    )

    val mecanum = MecanumController(drivebase,io)
    private val choreoController = ChoreoController(Tuning.choreoPosPID,Tuning.choreoAngPID,Physical.DRIVEBASE_SIZE.xy,0)
    val choreo = choreoControllerLoop(choreoController,this)
    val trajLogger = choreoController.trajectoryLogger
    val slides = slidesControlLoop(io,initSlidesPose)
    val arm = with(io) { ArmControlLoop(Actuator { armL = it }, Actuator { armR = it }, initArmPose, io) }

    val intake = transfer { d, x: Unit, t: Tuning.IntakePosition ->
        IntakeAngleKinematics.inverse(t.x)
    }.toControlLoop("intake",io,{},{
        io.intakeL = Limits.INTAKE_SERVO_1.toServoPos()(it); io.intakeR = Limits.INTAKE_SERVO_2.toServoPos()(it)
   }).also { it.t = intakePos  }

    val claw: Actuator<Double> = with(io) { Actuator { claw = it } }
    val active: Actuator<Double> = with(io) { Actuator({ Exception().printStackTrace() }) { intake = it }}
    val flap: Actuator<Double> = with(io) { Actuator { flap = it }}

    val extendCommandSlot = CommandSlot()
    val liftCommandSlot = CommandSlot()
    val armCommandSlot = CommandSlot()
    val intakeCommandSlot = CommandSlot()

    init {
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
        io.updateColor()
        io.updatePinpoint()
    }
}