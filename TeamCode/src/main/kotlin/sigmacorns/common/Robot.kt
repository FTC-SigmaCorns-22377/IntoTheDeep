package sigmacorns.common

import eu.sirotin.kotunil.core.div
import eu.sirotin.kotunil.core.times
import eu.sirotin.kotunil.derived.Volt
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math2.Tick
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.revolution
import net.unnamedrobotics.lib.rerun.RerunConnection
import net.unnamedrobotics.lib.rerun.RerunPrefix
import net.unnamedrobotics.lib.rerun.Rerunable
import net.unnamedrobotics.lib.rerun.rerun
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.subsystems.arm.ArmController
import sigmacorns.common.subsystems.arm.ArmState
import sigmacorns.common.subsystems.arm.ArmTarget
import sigmacorns.common.subsystems.arm.DiffyInputPose
import sigmacorns.common.subsystems.swerve.SwerveController
import sigmacorns.common.subsystems.swerve.SwerveTarget

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
    val nextState: ControllerState,
    val turnPowers: Array<Double>,
    val drivePowers: Array<Double>,
    val armPowers: List<Double>,
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
)

data class RobotTarget(
    val swerveTarget: SwerveTarget,
    val armTarget: ArmTarget
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
        .map { (it/Constants.ANALOG_MAX*revolution).cast(rad).value }

    val armPos = ArmState(DiffyInputPose(input.armMotor1Pos, input.armMotor2Pos))

    val swerveO = input.state.swerveController.updateStateless(dt,swerveState,target.swerveTarget)
    val armO = input.state.armController.updateStateless(dt,armPos,target.armTarget)

    return RobotTickO(
        turnPowers = swerveO.turnPowers,
        drivePowers = swerveO.drivePowers,
        armPowers = armO.motors.map { (it/input.v).value },
        nextState = ControllerState(
            lastT = input.t,
            input.state.swerveController,
            input.state.armController
        )
    )
}

fun nextState(io: SigmaIO, cur: RobotTickI, target: RobotTarget): RobotTickI {
    val o = pureTick(cur,target)

    rerun(io.rerunConnection) {
        log("cur") { cur }
        log("swerve") { cur.state.swerveController }
        log("arm") { cur.state.armController }
        log("outputs") { o }
    }

    return io.update(o)
}