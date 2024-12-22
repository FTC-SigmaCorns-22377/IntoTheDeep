package sigmacorns.common.swerve

//import net.unnamedrobotics.lib.command.Command
//import net.unnamedrobotics.lib.command.cmd
//import net.unnamedrobotics.lib.math.Angle
//import net.unnamedrobotics.lib.math.Measurement
//import net.unnamedrobotics.lib.math.inches
//import sigmacorns.common.Robot
//
//fun p2pCommand(robot: Robot, target: PositionTarget, pThreshold: Measurement, thetaThresh: Angle): Command {
//    return cmd {
//        init { robot.positionController.target = target }
//        finishWhen {
//            robot.odo.pose.distanceTo(target).inches < pThreshold && robot.odo.pose.angleDistanceTo(target) < thetaThresh
//        }
//    }
//}