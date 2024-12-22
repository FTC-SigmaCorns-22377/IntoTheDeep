package sigmacorns.common

import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.mm
import eu.sirotin.kotunil.base.ms
import eu.sirotin.kotunil.derived.V
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math.Vector2
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.tick
import net.unnamedrobotics.lib.rerun.rerun
import org.junit.Test
import sigmacorns.common.io.SimArmState
import sigmacorns.common.io.SimIO
import sigmacorns.common.subsystems.arm.ArmController
import sigmacorns.common.subsystems.arm.ArmState
import sigmacorns.common.subsystems.arm.ArmTarget
import sigmacorns.common.subsystems.arm.DiffyInputPose
import sigmacorns.common.subsystems.swerve.SwerveController
import sigmacorns.common.subsystems.swerve.SwerveTarget

class ArmTest {
    @Test
    fun simTest() {
        val initialArmPivot = 0.rad
        val initialArmExtension = 400.mm
        val io = SimIO(10.ms,12.V, SimArmState(initialArmPivot,initialArmExtension))

        println("RUNNING")
        io.use {
            var state = io.update(RobotTickO(
                ControllerState(0.0,SwerveController(), ArmController()),
                Array(4) { 0.0 },
                Array(4) { 0.0 },
                List(2) { 0.0 }
            ))

            state.state.armController.updateStateless(1.0, ArmState(DiffyInputPose(0.tick,0.tick)),
                ArmTarget(0.rad,0.m,0.rad,0.rad)
            )
            state.state.swerveController.updateStateless(1.0,List(4) { 0.0 },
                SwerveTarget(Vector2(),0.0,false)
            )

//            state = RobotTickI(nextState(io,state,
//                RobotTarget(
//                    SwerveTarget(Vector2(),0.0,false),
//                    ArmTarget(70.degrees,400.mm,0.rad,0.rad)
//                )
//            ).state,state.t,state.v,state.turnEncodersPos,state.armMotor1Pos,state.armMotor2Pos)


            for(i in 0..3000) {
                println("BEFORE MOVING ON")
                state = io.update(RobotTickO(state.state,Array(4) {0.0},Array(4) {0.0}, listOf(0.01,0.0)))

                rerun(io.rerunConnection) {
                    log("i") {state}
                }


                pureTick(state,RobotTarget(SwerveTarget(Vector2(),0.0,false),
                    ArmTarget(0.rad,400.m,0.rad,0.rad)
                ))

                rerun(io.rerunConnection) {
                    log("cur") { state }
                    log("swerve") { state.state.swerveController }
                    log("arm") { state.state.armController }
//                    log("outputs") { o }
                }
                println("step $i done")
            }
        }
    }


    @Test
    fun armControlTest() {
        val initialArmPivot = 0.rad
        val initialArmExtension = 400.mm
        val io = SimIO(10.ms,12.V, SimArmState(initialArmPivot,initialArmExtension))

        println("RUNNING")
        io.use {
            var state = io.update(RobotTickO(
                ControllerState(0.0,SwerveController(), ArmController()),
                Array(4) { 0.0 },
                Array(4) { 0.0 },
                List(2) { 0.0 }
            ))

            val target = RobotTarget(
                SwerveTarget(Vector2(),0.0,false),
                ArmTarget(70.degrees,400.mm,0.rad,0.rad)
            )

            for(i in 0..300) {
                state = nextState(io,state,target)

                rerun(io.rerunConnection) {
                    log("i") {state}
                }
            }
        }
    }
}