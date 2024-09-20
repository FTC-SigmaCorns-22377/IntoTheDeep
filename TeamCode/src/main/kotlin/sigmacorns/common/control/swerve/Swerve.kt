package sigmacorns.common.control.swerve

import android.icu.text.DecimalFormat
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.PIDCoefficients
import net.hivemindrobotics.lib.control.controller.PIDController
import net.hivemindrobotics.lib.math.Vector2
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.PI

class Swerve(val drives: Array<DcMotor>, var turns: Array<CRServo>, var turnEncoders: Array<DcMotor>) {
    val turnPIDCoefficients = PIDCoefficients(0.0001,0.0,0.0)
    val moduleController = ModuleController(
        PIDController(turnPIDCoefficients),
        0.0,
        Vector2(0,0)
    )

    val modules = arrayOf(moduleController.copy(),moduleController.copy(),moduleController.copy(),moduleController.copy())
    val reversedPowers = arrayOf(1.0,-1.0,1.0,1.0)
    val reversedEncoders = arrayOf(-1.0,-1.0,1.0,1.0)
    val reversedTurns = arrayOf(-1.0,-1.0,-1.0,-1.0)

    val turnDirs = arrayOf(
        Vector2.fromAngle(7* PI /4.0,1.0),
        Vector2.fromAngle(5* PI /4.0,1.0),
        Vector2.fromAngle(3* PI /4.0,1.0),
        Vector2.fromAngle(1* PI /4.0,1.0),
    )

    init {
        for(i in modules.indices) {
            modules[i].drivePowerReversed = reversedPowers[i]
            modules[i].reverseEncoders = reversedEncoders[i]
            modules[i].turnPowerReversed = reversedTurns[i]
        }
    }

    fun update(targetPower: Vector2, angVelPower: Double, dt: Double) {
        var vs = turnDirs.map { it*angVelPower + targetPower }
        val maxMag = vs.maxBy { it.magnitude }.magnitude
        if(maxMag>1.0) vs = vs.map { it/maxMag }

        for(i in modules.indices) {
            val powers = modules[i].update(dt, turnEncoders[i].currentPosition.toDouble(), vs[i])
            turns[i].power = powers.turn
            drives[i].power = powers.drive
        }

        if(telemetry!=null) {
            telemetry!!.addData("vs [0]",vs[0])
            telemetry!!.addData("vs [1]",vs[1])
            telemetry!!.addData("vs [2]",vs[2])
            telemetry!!.addData("vs [3]",vs[3])
        }
    }

    fun reset() {
        modules.forEach { it.target = ModuleTarget(0,0); it.position=0.0; }
        turnEncoders.forEach {
            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }

    var telemetry: Telemetry? = null;
    fun telemetry(tel: Telemetry) {
        telemetry = tel
        for(i in modules.indices) {
            val fmt = DecimalFormat("#,###.##")
            tel.addData("Wheel " + (i+1),
                "target = " + fmt.format(modules[i].target.angleFromOrigin)  +
                        " pos = " + fmt.format(modules[i].position) +
                        " power = " + fmt.format(turns[i].power) +
                        "\n angle = " + fmt.format(tickToAngle((reversedEncoders[i]*turnEncoders[i].currentPosition).toInt()))
            )
        }

    }
}