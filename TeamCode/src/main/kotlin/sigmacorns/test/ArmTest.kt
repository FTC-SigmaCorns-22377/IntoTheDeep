//package sigmacorns.test
//
//import com.acmerobotics.dashboard.FtcDashboard
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp
//import com.qualcomm.robotcore.hardware.Gamepad
//import eu.sirotin.kotunil.base.cm
//import eu.sirotin.kotunil.base.m
//import eu.sirotin.kotunil.base.mm
//import eu.sirotin.kotunil.core.*
//import eu.sirotin.kotunil.derived.rad
//import net.unnamedrobotics.lib.math2.Vector2
//import net.unnamedrobotics.lib.math2.cast
//import net.unnamedrobotics.lib.math2.degrees
//import net.unnamedrobotics.lib.math2.normalizeRadian
//import net.unnamedrobotics.lib.math2.vec2
//import net.unnamedrobotics.lib.rerun.rerun
//import sigmacorns.common.ControllerState
//import sigmacorns.common.RobotTarget
//import sigmacorns.common.RobotTickO
//import sigmacorns.common.io.RobotIO
//import sigmacorns.common.nextState
//import sigmacorns.common.subsystems.arm.ArmController
//import sigmacorns.common.subsystems.arm.ArmPose
//import sigmacorns.common.subsystems.arm.ArmTarget
//import sigmacorns.common.subsystems.arm.ClawTarget
//import sigmacorns.common.subsystems.arm.DiffyInputPose
//import sigmacorns.common.subsystems.arm.DiffyOutputPose
//import sigmacorns.common.subsystems.swerve.SwerveController
//import sigmacorns.common.subsystems.swerve.SwerveTarget
//import kotlin.math.absoluteValue
//
//@TeleOp
//class ArmTest: LinearOpMode() {
//    override fun runOpMode() {
//
//        val initialArmPivot = 0.rad
//        val initialArmExtension = 400.mm
//        val io = RobotIO(hardwareMap, io = "192.168.43.122", initialArmPose = ArmPose(vec2(),0.rad,initialArmExtension,initialArmPivot,0.rad,0.rad))
//        var state = io.initial()
//
//        var aPressed = false
//        var bPressed = false
//
//        var pivotToggle = false
//        var extendToggle = false
//        waitForStart()
//
//        println("RUNNING")
//        io.use {
//            val target = RobotTarget(
//                SwerveTarget(net.unnamedrobotics.lib.math.Vector2(),0.0,false),
//                ArmTarget(20.degrees,600.mm,0.rad,0.rad),
//                ClawTarget(0.rad,0.rad),
//                false
//            )
//
//            while (opModeIsActive()) {
//                state = nextState(io,state,target)
//
//                val dt = (state.t-state.state.lastT)/1000
////                gamepad1.type = Gamepad.Type.XBOX_360
//                target.armTarget.pivot = (target.armTarget.pivot + (gamepad1.left_stick_y*dt).rad).cast(rad)
//                target.armTarget.extension = (target.armTarget.extension + (gamepad1.right_stick_y*dt).m).cast(m)
//                if(gamepad1.a && !aPressed) {
//                    target.armTarget.pivot = if(pivotToggle) 70.degrees else 0.degrees
//                    pivotToggle = !pivotToggle
//                }
//                if(gamepad1.b && !bPressed) {
//                    target.armTarget.extension = if(extendToggle) 700.mm else 500.mm
//                    extendToggle = !extendToggle
//                }
//
//                aPressed = gamepad1.a
//                bPressed = gamepad1.b
//                rerun(io.rerunConnection) {
//                    log("state") { state }
//                }
//            }
//        }
//    }
//}