package sigmacorns.common.control

import dev.nullrobotics.sample.SwerveSample
import dev.nullrobotics.trajectory.Trajectory
import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.core.*
import net.unnamedrobotics.lib.math.RGBA
import net.unnamedrobotics.lib.math2.vec3
import net.unnamedrobotics.lib.rerun.RerunConnection
import net.unnamedrobotics.lib.rerun.RerunPrefix
import net.unnamedrobotics.lib.rerun.archetypes.Arrows3D
import net.unnamedrobotics.lib.rerun.archetypes.Boxes3D
import net.unnamedrobotics.lib.rerun.archetypes.Points3D
import org.joml.Quaterniond

class TrajectoryLogger(val length: Metre, val width: Metre, val downscale: Int = 1, val velScaling: Double = 0.1) {
    private val loggedTrajectories: HashMap<Trajectory<SwerveSample>, Mode> = hashMapOf()

    enum class Mode { BOXES, POINTS, VEL_ARROW, ACCEL_ARROW }

    context(RerunPrefix, RerunConnection)
    fun logTraj(traj: Trajectory<SwerveSample>, mode: Mode, name: String = "${traj.name()}.traj", ) {
        if(loggedTrajectories[traj]?.equals(mode) != true) {
            loggedTrajectories[traj] = mode

            log(name) {
                val samples = traj.samples.filterIndexed { i,_ -> i%downscale == 0 }
                val centers = samples.map { vec3(it.x.m,it.y.m,0.m) }
                val colors = samples.map { RGBA(1.0,1.0,1.0,1.0) }

                when(mode) {
                    Mode.BOXES -> Boxes3D(
                        halfSizes = List(samples.size) { vec3(length/2.0,width/2.0,0.m) },
                        centers = centers,
                        colors = colors
                    )
                    Mode.POINTS -> Points3D(origins = centers)
                    Mode.VEL_ARROW -> Arrows3D(
                        origins = centers,
                        vecs = samples.map { vec3(it.vx, it.vy, 0)*velScaling },
                        colors = colors
                    )
                    Mode.ACCEL_ARROW -> Arrows3D(
                        origins = centers,
                        vecs = samples.map { vec3(it.ax, it.ay, 0) },
                        colors = colors
                    )
                }
            }
        }
    }

    context(RerunPrefix, RerunConnection)
    fun highlightSample(name: String, s: SwerveSample) {
        log(name) {
            Boxes3D(
                halfSizes = listOf( vec3(length/2.0,width/2.0,0.m) ),
                centers = listOf(vec3(s.x,s.y,0)),
                rotations = listOf(Quaterniond().setAngleAxis(s.heading,0.0,0.0,1.0))
            )
        }
    }
}



