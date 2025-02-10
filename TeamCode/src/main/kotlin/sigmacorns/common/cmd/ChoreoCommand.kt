package sigmacorns.common.cmd

import dev.nullrobotics.Choreo
import dev.nullrobotics.sample.SwerveSample
import dev.nullrobotics.trajectory.Trajectory
import net.unnamedrobotics.lib.command.cmd
import sigmacorns.common.Robot

fun choreoCommand(robot: Robot, trajName: String) = robot.choreo.follow(Choreo.loadTrajectory<SwerveSample>(trajName).get())