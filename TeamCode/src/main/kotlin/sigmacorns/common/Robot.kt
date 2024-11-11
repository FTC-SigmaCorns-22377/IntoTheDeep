package sigmacorns.common

import com.qualcomm.robotcore.hardware.HardwareMap
import net.unnamedrobotics.lib.control.controller.params.PIDCoefficients
import net.unnamedrobotics.lib.hardware.impl.OctoEncoderImpl
import net.unnamedrobotics.lib.hardware.interfaces.IMU
import net.unnamedrobotics.lib.math.Pose
import net.unnamedrobotics.lib.util.Clock
import sigmacorns.common.hardware.RobotHardware
import sigmacorns.common.swerve.PositionController
import sigmacorns.common.swerve.SwerveController
import sigmacorns.common.swerve.TwoWheelOdo

class Robot(val hardware: RobotHardware) {
    val swerveModulePID = PIDCoefficients(0.00004,0.0,0.0)
    
    val odo = TwoWheelOdo(
        xEncoder = OctoEncoderImpl(hardware.octo,4),
        yEncoder = OctoEncoderImpl(hardware.octo,5),
        imu = hardware.imu,
        pose = Pose()
    )
    val swerveController = SwerveController(swerveModulePID)
    val positionController = PositionController()
    var lastT = Clock.milliseconds

    fun tick() {
        odo.update()

        val curT = Clock.milliseconds

        swerveController.target = positionController.output
        // TODO: UPDATE MODULE POSITION
        swerveController.update(curT-lastT)

        for(pair in hardware.turns.zip(swerveController.output.turnPowers)){
            pair.first.power = pair.second
        }

        for(pair in hardware.drives.zip(swerveController.output.drivePowers)){
            pair.first.power = pair.second
        }

        lastT = curT
    }

}