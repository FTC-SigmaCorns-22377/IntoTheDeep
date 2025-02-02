package sigmacorns.common.control

import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.physics.MecanumDrivebase
import sigmacorns.common.io.SigmaIO
import kotlin.math.max
import kotlin.math.min

// TODO(after qual): combine all the controllers into one control graph...
class MecanumController(
    drivebase: MecanumDrivebase,
    io: SigmaIO
): ControlLoop<Unit, List<Double>, Transform2D>("drivebase",io) {
    val kinematics = drivebase.powerKinematics
    override fun update(deltaTime: Double): List<Double> = kinematics.inverse(t.log())

    override var x: Unit = Unit
    override var u: List<Double> = List(4) { 0.0 }
    override var t: Transform2D = Transform2D(0.m/s, 0.m/s, 0.rad/s)

    override fun read() {}

    override fun reached(x: Unit, t: Transform2D): Boolean = true

    override fun write(u: List<Double>) {
        val scalar = max(u.max(),1.0)
        io.driveFL = u[0]/scalar
        io.driveBL = u[1]/scalar
        io.driveBR = u[2]/scalar
        io.driveFR = u[3]/scalar
    }
}