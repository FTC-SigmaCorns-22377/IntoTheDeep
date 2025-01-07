package sigmacorns.common

import eu.sirotin.kotunil.core.div
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.Volt
import net.unnamedrobotics.lib.math2.Tick
import net.unnamedrobotics.lib.rerun.RerunConnection
import net.unnamedrobotics.lib.rerun.RerunPrefix
import net.unnamedrobotics.lib.rerun.Rerunable
import net.unnamedrobotics.lib.rerun.rerun
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.subsystems.arm.ArmController
import sigmacorns.common.subsystems.arm.ArmState
import sigmacorns.common.subsystems.arm.ArmTarget
import sigmacorns.common.subsystems.arm.ClawController
import sigmacorns.common.subsystems.arm.ClawInput
import sigmacorns.common.subsystems.arm.ClawTarget
import sigmacorns.common.subsystems.arm.DiffyInputPose
import sigmacorns.common.subsystems.swerve.SwerveController
import sigmacorns.common.subsystems.swerve.SwerveTarget
import kotlin.math.PI

data class RobotTickI(
    val state: ControllerState,
    val t: Double,
    val v: Volt,
    val turnEncodersPos: List<Volt>,
    val armMotor1Pos: Tick,
    val armMotor2Pos: Tick,
): Rerunable {
    context(RerunPrefix, RerunConnection) override fun log(name: String) {
        scalar("time (ms)",t)
        scalar("voltage (V)",v.value)
        prefix("swerve") {
            prefix("turnEncoders") {
                turnEncodersPos.mapIndexed { i, volt -> scalar("turn${i}Encoder (V)",volt.value) }
            }
        }
        prefix("arm") {
            prefix("motorPos") {
                scalar("armMotor1Pos",armMotor1Pos.value)
                scalar("armMotor2Pos",armMotor2Pos.value)
            }
        }
    }
}

data class RobotTickO(
    var nextState: ControllerState,
    var turnPowers: Array<Double>,
    var drivePowers: Array<Double>,
    var armPowers: List<Double>,
    var diffyClawPos: ClawInput,
    var claw: Double
): Rerunable {
    context(RerunPrefix, RerunConnection) override fun log(name: String) {
        prefix("swerve") {
            prefix("turnPowers") {
                turnPowers.mapIndexed { i,it -> scalar("turnPower${i}",it) }
            }
            prefix("drivePowers") {

                drivePowers.mapIndexed { i,it -> scalar("drivePower${i}",it) }
            }
        }
        prefix("arm") {
            prefix("motorPowers")  {
               armPowers.mapIndexed { i,it -> scalar("armPower${i}",it) }
            }
        }
    }
}

data class ControllerState(
    val lastT: Double,
    val swerveController: SwerveController,
    val armController: ArmController,
    val clawController: ClawController
) {
    constructor(): this(0.0, SwerveController(), ArmController(), ClawController())
}

data class RobotTarget(
    var swerveTarget: SwerveTarget,
    var armTarget: ArmTarget,
    var clawTarget: ClawTarget,
    var clawClosed: Boolean
)

/**
 * A **pure** function to generate the output (hardware commands) of the robot
 * given:
 * - the results of hardware IO
 * - states of the controllers (I don't feel like implementing PID recursively)
 * - the target of the swerve drive
 * - the target of the arm
 */
fun pureTick(
    input: RobotTickI,
    target: RobotTarget,
): RobotTickO {
    val dt = input.t-input.state.lastT
    val swerveState = input.turnEncodersPos
        .map { (it/Constants.ANALOG_MAX)* PI *2 }.zip(Constants.ENCODER_OFFSETS).map { (it.first-it.second).value }

    val armPos = ArmState(DiffyInputPose(input.armMotor1Pos, input.armMotor2Pos))

    val swerveO = input.state.swerveController.updateStateless(dt,swerveState,target.swerveTarget)
    val armO = input.state.armController.updateStateless(dt,armPos,target.armTarget)
    val clawO = input.state.clawController.updateStateless(dt,Unit,target.clawTarget)

    return RobotTickO(
        turnPowers = swerveO.turnPowers,
        drivePowers = swerveO.drivePowers,
        armPowers = armO.motors.map { (it/input.v).value },
        diffyClawPos = clawO,
        claw = if(target.clawClosed) Constants.CLAW_CLOSED else Constants.CLAW_OPEN,
        nextState = ControllerState(
            lastT = input.t,
            input.state.swerveController,
            input.state.armController,
            input.state.clawController
        )
    )
}

var TICK_NUMBER = 0
const val RERUN_RES = 10

fun nextState(io: SigmaIO, cur: RobotTickI, target: RobotTarget): RobotTickI {
    TICK_NUMBER += 1
    val o = pureTick(cur,target)

//    o.armPowers = o.armPowers.map { it.clampMagnitude(0.3) }

    if(TICK_NUMBER% RERUN_RES == 0) rerun(io.rerunConnection) {
        log("cur") { cur }
        log("swerve") { cur.state.swerveController }
        log("arm") { cur.state.armController }
        log("outputs") { o }
    }

    return io.update(o)
}