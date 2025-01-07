package sigmacorns.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.core.plus
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math.Vector2
import net.unnamedrobotics.lib.math2.map
import net.unnamedrobotics.lib.math2.normalize
import net.unnamedrobotics.lib.math2.vec2
import sigmacorns.common.RobotTarget
import sigmacorns.common.io.RobotIO
import sigmacorns.common.nextState
import sigmacorns.common.subsystems.arm.ArmTarget
import sigmacorns.common.subsystems.arm.ClawTarget
import sigmacorns.common.subsystems.swerve.SwerveTarget

@TeleOp
class ClawTest: LinearOpMode() {
    override fun runOpMode() {
        val io = RobotIO(hardwareMap)
        var state = io.initial()

        var pitch = 0.rad
        var roll = 0.rad

        waitForStart()

        while (opModeIsActive()) {
            val dt = (state.t-state.state.lastT)/10
            pitch = pitch.map { it + dt*gamepad1.left_stick_y  }
            roll = roll.map { it + dt*gamepad1.left_stick_x  }

            println("pitch = $pitch")
            println("roll = $roll")
            state = nextState(io,state, RobotTarget(SwerveTarget(Vector2(),0.0,false), ArmTarget(Double.NaN.rad,Double.NaN.m,pitch,roll),
                ClawTarget(pitch,roll),false
            ))
        }
    }
}