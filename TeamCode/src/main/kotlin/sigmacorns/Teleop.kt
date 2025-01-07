package sigmacorns

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.mm
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math.Vector2
import net.unnamedrobotics.lib.math.radians
import net.unnamedrobotics.lib.math2.Bounds
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.inches
import net.unnamedrobotics.lib.math2.map
import net.unnamedrobotics.lib.math2.vec2
import net.unnamedrobotics.lib.math2.vec3
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians
import sigmacorns.common.Constants
import sigmacorns.common.ControllerState
import sigmacorns.common.RobotTarget
import sigmacorns.common.io.RobotIO
import sigmacorns.common.nextState
import sigmacorns.common.subsystems.arm.ArmController
import sigmacorns.common.subsystems.arm.ArmPose
import sigmacorns.common.subsystems.arm.ArmTarget
import sigmacorns.common.subsystems.arm.ClawTarget
import sigmacorns.common.subsystems.arm.ScoringKinematics
import sigmacorns.common.subsystems.arm.ScoringTarget
import sigmacorns.common.subsystems.swerve.SwerveController
import sigmacorns.common.subsystems.swerve.SwerveTarget
import kotlin.math.PI

//enum class Mode {
//    PICKUP,
//    TRANSITION,
//
//}

@TeleOp
class Teleop: LinearOpMode() {
    override fun runOpMode() {
        val initialArmExtension = 400.mm
        val initialArmPivot = (80).degrees

        val io = RobotIO(hardwareMap, io = "192.168.43.122", initialArmPose = ArmPose(vec2(),0.rad,initialArmExtension,initialArmPivot,0.rad,0.rad))

        io.rerunConnection.disabled = true
        var state = io.initial()

        val target = RobotTarget(
            SwerveTarget(Vector2(),0.0,false),
            ArmTarget(initialArmPivot,initialArmExtension,0.rad,0.rad),
            ClawTarget((-83).rad,0.rad),
            false
        )

        waitForStart()

        var extension = initialArmExtension
        var pivot = initialArmPivot
        var roll = 0.rad
        var pickup = false
        var open = false
        var wasAPressed = false
        var wasBPressed = false

        while (opModeIsActive()) {
//            val pickupPos = ScoringKinematics.inverse(ScoringTarget(
//                vec2(0.m,0.m),
//                0.rad,
//                vec3(extension,0.m,pivot),
//                0.rad,
//                (-90).degrees
//            ))

            val dt = (state.t-state.state.lastT)/1000

            if (gamepad2.a && !wasAPressed) {
                pickup = !pickup
                if(!pickup) roll = 60.degrees else roll = 0.degrees
            }
            if (gamepad2.b && !wasBPressed) open = !open
            wasAPressed = gamepad2.a
            wasBPressed = gamepad2.b

            extension = (extension + (-gamepad2.right_stick_y*dt*.8).m).cast(m)
            extension = Constants.ARM_EXTENSION_BOUNDS.apply(extension)
//            extension = Bounds(400.mm,1000.mm).apply(extension)

            pivot = (pivot + (gamepad2.left_stick_y*dt*1.5).rad).cast(rad)
            pivot = Constants.ARM_PIVOT_BOUNDS.apply(pivot)
//            height = Bounds(1.inches,32.inches).apply(height)

            roll = (normalizeRadians(roll.value + (if(gamepad2.right_bumper) 1.0 else 0.0 - (if(gamepad2.left_bumper) 1.0 else 0.0))*dt).rad)

            val pitch = if(pickup) (83.rad) else (-33).rad
//            println("pickup pos extension = ${pickupPos.extension}, pivot = ${pickupPos.pivot}")


            target.swerveTarget.transform = Vector2(-gamepad1.left_stick_y, -gamepad1.left_stick_x)
            target.swerveTarget.turn = gamepad1.right_stick_x.toDouble()*0.5

            target.armTarget = ArmTarget(pivot,extension,pitch,0.degrees)
            target.clawTarget = ClawTarget(pitch,roll.map { it/(2* PI)*360 })
            target.clawClosed = !open

            state = nextState(io,state,target)
        }
    }
}