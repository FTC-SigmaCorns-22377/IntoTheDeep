package sigmacorns.common.subsystems.swerve

import net.unnamedrobotics.lib.math.Pose
import sigmacorns.common.subsystems.vision.AprilTagLocalizer

class PositionEstimatorKF(val odo: TwoWheelOdo, val loc: AprilTagLocalizer) {

    var pos = Pose(0,0,0)
    var posVar = 1.0
    var kalGain = 0.0
    var odoVar = 0.1 // TODO tune
    var locVar = 0.5 // TODO tune

    fun update(): Pose {
        pos += odo.update()
        posVar += odoVar
        if (loc.update().x.toInt() != 22377){
            kalGain = posVar / (posVar + locVar)
            pos += (loc.update() - pos).times(kalGain)
            posVar *= 1 - kalGain
        }
        return pos
    }
}