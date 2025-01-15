package sigmacorns.common

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import eu.sirotin.kotunil.base.Second
import eu.sirotin.kotunil.base.kg
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.mm
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.derived.N
import eu.sirotin.kotunil.derived.rad
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.inches
import net.unnamedrobotics.lib.physics.LinearTireModel
import net.unnamedrobotics.lib.physics.SwerveDrivebase
import net.unnamedrobotics.lib.physics.goBildaMotorConstants
import net.unnamedrobotics.lib.rerun.rerun
import net.unnamedrobotics.lib.util.Clock
import sigmacorns.common.io.ControlLoopContext
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.subsystems.arm.armControlLoop
import sigmacorns.common.subsystems.swerve.ModuleController
import sigmacorns.common.subsystems.swerve.SwerveController
import sigmacorns.common.subsystems.swerve.swerveControlLoop
import sigmacorns.common.subsystems.swerve.swerveLogPosControlLoop
import sigmacorns.common.subsystems.swerve.swerveLogVelControlLoop
import sigmacorns.opmode.SIM
import java.io.Closeable

class Robot(val io: SigmaIO): Closeable {
    val drivebase = SwerveDrivebase(
        0.048.m,
        18.inches,
        18.inches,
        20.kg,
        31.5.kg* mm * mm *15,
        18.5.kg* mm * mm,
        turnMotor = goBildaMotorConstants(6000.0/435.0),
        driveMotor = goBildaMotorConstants(6000.0/435.0),
        tireModel = LinearTireModel(10.0,1.0,1.0,30.N,30.N,3.N*m)
    )

    var ioLoop: Job? = null

    val armControlLoop = armControlLoop()
    val swerveControlLoop = swerveControlLoop(SwerveController(
        ModuleController(Tuning.SWERVE_MODULE_PID),
        drivebase)
    )
    val swerveLogPosControlLoop = swerveLogPosControlLoop(swerveControlLoop)
    val swerveLogVelControlLoop = swerveLogVelControlLoop(swerveControlLoop)

    context(LinearOpMode)
    fun launchIOLoop() {
        ioLoop = CoroutineScope(Dispatchers.Default).launch { ioLoop { !opModeIsActive() } }
    }

    var lastT: Second = io.time()

    context(LinearOpMode)
    fun inputLoop(f: (Second) -> Unit) {
        try {
            use {
                while (opModeIsActive()) {
                    val t = io.time()
                    val dt = (t-lastT).cast(s)
                    lastT = t
                    f(dt)
                    if(SIM) Thread.sleep(50)
                }
            }
        } catch (e: Exception) {
            runBlocking { ioLoop?.cancelAndJoin() }
            throw e
        }
    }

    fun launchIOLoop(done: (Second) -> Boolean) {
        ioLoop = CoroutineScope(Dispatchers.Default).launch { ioLoop(done)  }
    }

    fun addLoop(controlLoopContext: ControlLoopContext<*,*,*,SigmaIO,*>) {
        io.addLoop(controlLoopContext)
    }

    suspend fun ioLoop(done: (Second) -> Boolean) {
        var t = io.time()
        while (!done(t)) {
            t = io.time()
            io.run(t)
        }
    }

    override fun close() {
        ioLoop?.cancel()
        io.rerunConnection.close()
    }

    fun topSpeed(): Expression = drivebase.driveMotor.topSpeed(1.0)*drivebase.radius
    fun topAngularSpeed(): Expression {
        val tangentialSpeed = topSpeed()
        return drivebase.wheels.minOf {
            tangentialSpeed/it.position.magnitude()
        }
    }
}