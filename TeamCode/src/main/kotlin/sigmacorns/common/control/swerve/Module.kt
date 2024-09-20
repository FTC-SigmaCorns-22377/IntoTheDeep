package sigmacorns.common.control.swerve

import net.hivemindrobotics.lib.control.controller.GeneralController
import net.hivemindrobotics.lib.control.controller.PIDController
import net.hivemindrobotics.lib.math.Vector2
import kotlin.math.PI
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians
import kotlin.math.absoluteValue
import kotlin.math.sign

const val TICKS_PER_REV: Double = 20480.0

//the only state controllers should have is dt
//guard against encoder wraparound in pid controller.
data class ModuleInput(var turn: Double, var drive: Double)
typealias ModuleState = Double // turn encoder ticks
typealias ModuleTarget = Vector2 // direction + speed wheel should be spinning at.

//The job of the ModuleController is to convert target
class ModuleController(
    val turnController: PIDController,
    override var position: ModuleState,
    override var target: ModuleTarget
): GeneralController<ModuleState,ModuleTarget,ModuleInput>() {

    //either -1 or 1
    var motorsFlipped = 1.0
    var reverseEncoders = 1.0
    var drivePowerReversed = 1.0
    var turnPowerReversed = 1.0
    override fun update(dt: Double): ModuleInput {
        //convert encoder ticks to module angle (radians)
        val p = position*reverseEncoders
        val curAngle = tickToAngle(p.toInt())*motorsFlipped
        var diff = normalizeRadians(target.angleFromOrigin - curAngle)

        //when targeting an angle > 90 degrees away it is better to reverse the motors and turn less.
        if (diff.absoluteValue > PI/2.0) {
            diff = (diff.absoluteValue-PI)*diff.sign
            motorsFlipped*=-1
        }

        //convert from angle back to encoder ticks
        val targetTicks = p + diff/(2*PI) * TICKS_PER_REV

        val turnPow = turnController.update(dt,p,targetTicks)

        return ModuleInput(turnPowerReversed*turnPow, target.magnitude*motorsFlipped*drivePowerReversed)
    }

    override fun copy() = ModuleController(turnController,position,target)
}

fun tickToAngle(pos: Int): Double {
    //-2pi to 2pi
    val a = 2*PI*((pos.toDouble()/ TICKS_PER_REV)%1.0)
    //-pi to pi
    return normalizeRadians(a)
}

//not used in ModuleController, useful in BasicSwerveTest.
fun angleToTick(curPos: Double, desiredHeading: Double): Double {
    val curAngle = tickToAngle(curPos.toInt())
    val diff = normalizeRadians(desiredHeading-curAngle)
    return curPos + diff/(2*PI) * TICKS_PER_REV
}