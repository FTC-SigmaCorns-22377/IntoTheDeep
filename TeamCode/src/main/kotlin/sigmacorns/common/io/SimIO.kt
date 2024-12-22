package sigmacorns.common.io

import eu.sirotin.kotunil.base.A
import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.Second
import eu.sirotin.kotunil.base.kg
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.s
import eu.sirotin.kotunil.core.Expression
import eu.sirotin.kotunil.core.div
import eu.sirotin.kotunil.core.minus
import eu.sirotin.kotunil.core.plus
import eu.sirotin.kotunil.core.pow
import eu.sirotin.kotunil.core.times
import eu.sirotin.kotunil.core.unaryMinus
import eu.sirotin.kotunil.derived.N
import eu.sirotin.kotunil.derived.Radian
import eu.sirotin.kotunil.derived.V
import eu.sirotin.kotunil.derived.Volt
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.tick
import net.unnamedrobotics.lib.physics.RK45Integrator
import net.unnamedrobotics.lib.physics.State
import net.unnamedrobotics.lib.rerun.RerunConnection
import net.unnamedrobotics.lib.rerun.rerun
import sigmacorns.common.Constants
import sigmacorns.common.RobotTickI
import sigmacorns.common.RobotTickO
import sigmacorns.common.subsystems.arm.DiffyKinematics
import sigmacorns.common.subsystems.arm.DiffyOutputPose
import kotlin.math.cos
import kotlin.math.sin

class SimIO(
    val updateTime: Second,
    val simV: Volt,
    var armState: SimArmState
): SigmaIO {
    override val rerunConnection = RerunConnection("lambda","127.0.0.1")

    private val boxTubeKinematics = DiffyKinematics(Constants.ARM_PIVOT_RATIO,Constants.ARM_EXTENSION_RATIO)
    private val integrator: RK45Integrator<SimArmState, List<Volt>> = RK45Integrator(
        tolerance = 1e-8,
        minStep = 1e-7
    )

    override fun update(o: RobotTickO): RobotTickI {
        val dx = { _: Double, x: SimArmState, u: List<Volt> -> this.simArm(x,u) }

        val lastT = o.nextState.lastT
        val nextT = lastT + updateTime.value
        val u =o.armPowers.map { (it*simV).cast(V) }

        val steps = integrator.integrate(dx,lastT,nextT,armState,u)
        armState = steps.second.last()

        simArm(armState,u,true)

        val armMotorPos = boxTubeKinematics.inverse(DiffyOutputPose(armState.pivot, armState.extension))

        rerun(rerunConnection) {
            prefix("sim") {
                scalar("pivot",armState.pivot.value)
                scalar("extend",armState.extension.value)
                scalar("vPivot",armState.vPivot.value)
                scalar("vExtend",armState.vExtend.value)
                scalar("i1",armState.i1.value)
                scalar("i2",armState.i2.value)
            }
        }

        return RobotTickI(
            state = o.nextState,
            t = nextT,
            v = simV,
            turnEncodersPos = List(4) { 0.V },
            armMotor1Pos = armMotorPos.axis1.cast(tick),
            armMotor2Pos = armMotorPos.axis2.cast(tick)
        )
    }

    private val pivotRatio = (Constants.ARM_PULLEY_RATIO.pow(-1) * Constants.ARM_MOTOR_GEAR_RATIO).cast(rad/rad)
    private val extendRatio = ((1.m/2.m) / Constants.ARM_SPOOL_RADIUS / Constants.ARM_DIFFY_RATIO * pivotRatio)
    fun simArm(x: SimArmState, u: List<Volt>, log: Boolean = false): SimArmState {
        val Kt = Constants.MOTOR_TORQUE_CONSTANT
        val inductance = Constants.MOTOR_INDUCTANCE
        val R = Constants.MOTOR_RESISTANCE

        val armWeight = 1 * kg
        val armSlidingWeight = 1*kg

        val v1 = ((x.vPivot*pivotRatio+x.vExtend*extendRatio)).cast(rad/s)
        val v2 = ((x.vPivot*pivotRatio-x.vExtend*extendRatio)).cast(rad/s)

        val di1 = (-Kt*v1 - R*x.i1 + u[0])/inductance
        val di2 = (-Kt*v2 - R*x.i2 + u[1])/inductance

        val t1 = (Kt*x.i1).cast(N*m)
        val t2 = (Kt*x.i2).cast(N*m)

        val armMoment = x.extension * Constants.ARM_MOMENT_RATIO

        val g = (-9.8).m/s/s

        //pivot torques
        val pivotTorque = (t1+t2)*pivotRatio
        val gravityTorque = -(sin(x.pivot.value)*armWeight*g*Constants.ARM_COM_DIST_TO_PIVOT).cast(N*m)

        // extension forces
        val fExtend = (t1-t2)*extendRatio

        val fCentrifugal = (x.vPivot.pow(2) * x.extension * armWeight).cast(N)*0
        val fGravity = (cos(x.pivot.value)*armSlidingWeight * g).cast(N)

        val dw = (pivotTorque + gravityTorque)/armMoment
        val dv = (fExtend + fCentrifugal + fGravity)/armWeight


        if(log) rerun(rerunConnection) {
            prefix("sim/dx") {
                scalar("v1",v1.value)
                scalar("v2",v2.value)
                scalar("fExtend",fExtend.value)
                scalar("t1",t1.value)
                scalar("t2",t2.value)
                scalar("dExtend",dv.value)
                scalar("di1",di1.value)
                scalar("di2",di2.value)
                scalar("gravityTorque", gravityTorque.value)
                scalar("gravityForce", fGravity.value)
            }
        }
        //validates units
        di1.cast(A/s)
        di2.cast(A/s)
        dw.cast(rad/s/s)
        dv.cast(m/s/s)

        return SimArmState(di1,di2,dw,dv,x.vPivot,x.vExtend)
    }
}

class SimArmState(
    val i1: Expression,
    val i2: Expression,
    val vPivot: Expression,
    val vExtend: Expression,
    val pivot: Expression,
    val extension: Expression
): State<SimArmState> {
    override var x: SimArmState = this

    constructor(pivot:Radian, extension:Metre) : this(0.A,0.A,0.rad/s,0.m/s, pivot, extension)

    override fun fromComponents(cs: List<Number>) = SimArmState(
        cs[0].A,cs[1].A,cs[2].rad/s,cs[3].m/s,cs[4].rad,cs[5].m
    )

    override fun toComponents() = listOf(
        i1.value,i2.value,vPivot.value,vExtend.value,pivot.value,extension.value
    )
}


class RungeKutta4(
    var state: SimArmState,
    private val dx: (Double,SimArmState,List<Volt>) -> SimArmState
) {
    fun step(input: List<Volt>, dt: Double) {
        //clones are needed because java arrays are pass-by-reference.
        val f1 = dx(0.0,state, input).toComponents().toDoubleArray()
        val stateF1 = state.toComponents().toDoubleArray()
        smallStep(stateF1, f1, dt / 2.0)
        val f2 = dx(0.0,state.fromComponents(stateF1.toList()), input).toComponents().toDoubleArray()
        val stateF2 = state.toComponents().toDoubleArray()
        smallStep(stateF2, f2, dt / 2.0)
        val f3 = dx(0.0,state.fromComponents(stateF2.toList()), input).toComponents().toDoubleArray()
        val stateF3 = state.toComponents().toDoubleArray()
        smallStep(stateF3, f3, dt)
        val f4 = dx(0.0,state.fromComponents(stateF3.toList()), input).toComponents()

        for (i in stateF3.indices) {
            stateF3[i] += (dt / 6.0) * (f1[i] + 2.0 * f2[i] + 2.0 * f3[i] + f4[i])
        }

        state = state.fromComponents(stateF3.toList())
    }

    private fun smallStep(initial: DoubleArray, dx: DoubleArray, dt: Double) {
        for (i in initial.indices) {
            initial[i] += dx[i] * dt
        }
    }
}
