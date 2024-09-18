package sigmacorns.common.control.swerve

import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.util.ElapsedTime
import net.hivemindrobotics.lib.control.controller.Controller
import net.hivemindrobotics.lib.control.filter.SlewRateLimiter
import kotlin.math.PI
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians
import sigmacorns.common.control.PIDAngleController

const val TICKS_PER_REV: Double = 20480.0

class Module(
    val motor: DcMotorEx,
    val turn: CRServoImplEx,
    val controller: Controller,
    val slewRateLimiter: SlewRateLimiter,
    val reverseEncoders: Boolean,
    val reversePower: Boolean,
) {
    private val timer = ElapsedTime()
    var target: Double = 0.0

    fun update() {
        val dt = timer.seconds()
        val pidTarget = slewRateLimiter.calculate(target)
    }
}

fun genSetpoint(curPos: Int, desiredHeading: Double): Int {
    //0-360
    val heading = 360*((curPos.toDouble()/ TICKS_PER_REV)%1.0)

    val delta = (desiredHeading - heading + 540) % 360 - 180;

    return (delta* TICKS_PER_REV).toInt()
}


//-pi..pi
fun tickToAngle(pos: Int): Double {
    //-2pi to 2pi
    val a = 2*PI*((pos.toDouble()/ TICKS_PER_REV)%1.0)
    //-pi to pi
    return normalizeRadians(a)
}

fun angleToTick(curPos: Double, desiredHeading: Double): Double {
    val curAngle = tickToAngle(curPos.toInt())
    val diff = normalizeRadians(desiredHeading-curAngle)
    val diffWrapped = if(diff>PI) diff-2*PI else if(diff<-PI) diff+2*PI else diff

    return diffWrapped/(2*PI) * TICKS_PER_REV
}