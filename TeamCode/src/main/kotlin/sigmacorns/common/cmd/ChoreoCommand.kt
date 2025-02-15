package sigmacorns.common.cmd

import dev.nullrobotics.Choreo
import dev.nullrobotics.sample.SwerveSample
import dev.nullrobotics.trajectory.Trajectory
import net.unnamedrobotics.lib.command.cmd
import net.unnamedrobotics.lib.command.groups.then
import sigmacorns.common.Robot

fun choreoCommand(robot: Robot, trajName: String) =
    robot.followPath(Choreo.loadTrajectory<SwerveSample>(trajName).get()).name("follow($trajName)")

fun fastChoreoCommand(robot: Robot, trajName: String) =
    robot.followPath(Choreo.loadTrajectory<SwerveSample>(trajName).get(), true).name("follow($trajName)")
