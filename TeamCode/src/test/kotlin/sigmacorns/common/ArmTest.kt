package sigmacorns.common

import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.mm
import eu.sirotin.kotunil.base.ms
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.derived.V
import eu.sirotin.kotunil.derived.rad
import kotlinx.coroutines.runBlocking
import net.unnamedrobotics.lib.math.Vector2
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.tick
import net.unnamedrobotics.lib.math2.vec2
import net.unnamedrobotics.lib.math2.vec3
import net.unnamedrobotics.lib.rerun.rerun
import org.junit.Test
import sigmacorns.common.io.SimIO
import sigmacorns.common.subsystems.arm.ArmTarget
import sigmacorns.common.subsystems.arm.ScoringKinematics
import sigmacorns.common.subsystems.arm.ScoringTarget

class ArmTest {
    @Test
    fun test() {
        val initialArmPivot = 0.rad
        val initialArmExtension = 400.mm
        val robot = Robot(SimIO(12.V,
            Transform2D(0.m,0.m,0.rad),
            initialArmPivot,
            initialArmExtension,
        ))

        robot.addLoop(robot.armControlLoop)

        robot.use {
            robot.launchIOLoop { it > 10.s }

            runBlocking {
                while (true) {
                    val t = robot.io.time()

                    robot.armControlLoop.target(ScoringKinematics.inverse(ScoringTarget(
                        vec2(0.m,0.m), 0.rad, vec3(30.cm,0.m,30.cm),0.rad,0.rad
                    )).let { ArmTarget(it.pivot,it.extension,it.pitch,it.roll,true) })

                    if(t > 10.s) break
                }
            }
        }
    }
}