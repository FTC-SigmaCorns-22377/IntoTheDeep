package sigmacorns.common.cmd

import dev.nullrobotics.Choreo
import dev.nullrobotics.sample.SwerveSample
import sigmacorns.common.Robot

fun choreoCommand(robot: Robot, trajName: String) =
    robot.followPath(Choreo.loadTrajectory<SwerveSample>(trajName).get()).also { it.name = "follow($trajName)" }

fun fastChoreoCommand(robot: Robot, trajName: String) =
    robot.followPath(Choreo.loadTrajectory<SwerveSample>(trajName).get(), true).also { it.name = "follow($trajName)" }
