package sigmacorns.common.io

import eu.sirotin.kotunil.base.A
import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.Second
import eu.sirotin.kotunil.base.kg
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.ms
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
import net.unnamedrobotics.lib.math2.Vector2
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.map
import net.unnamedrobotics.lib.math2.normalizeRadian
import net.unnamedrobotics.lib.math2.orthogonal
import net.unnamedrobotics.lib.math2.polar
import net.unnamedrobotics.lib.math2.tick
import net.unnamedrobotics.lib.math2.vec2
import net.unnamedrobotics.lib.physics.MotorState
import net.unnamedrobotics.lib.physics.RK45Integrator
import net.unnamedrobotics.lib.physics.State
import net.unnamedrobotics.lib.physics.goBildaMotorConstants
import net.unnamedrobotics.lib.rerun.RerunConnection
import net.unnamedrobotics.lib.rerun.rerun
import net.unnamedrobotics.lib.util.Clock
import sigmacorns.common.Constants
import sigmacorns.common.ControllerState
import sigmacorns.common.RobotTickI
import sigmacorns.common.RobotTickO
import sigmacorns.common.subsystems.arm.ArmPose
import sigmacorns.common.subsystems.arm.DiffyKinematics
import sigmacorns.common.subsystems.arm.DiffyOutputPose
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

class SimIO(
    val updateTime: Second = 10.ms,
    val simV: Volt = 12.V,
    var armState: SimArmState,
    val initialArmPose: ArmPose? = null
): SigmaIO {
    override val rerunConnection = RerunConnection("lambda","127.0.0.1")

    private val boxTubeKinematics = DiffyKinematics(Constants.ARM_PIVOT_RATIO,Constants.ARM_EXTENSION_RATIO)
    private val integrator: RK45Integrator<SimArmState, List<Volt>> = RK45Integrator(
        tolerance = 1e-8,
        minStep = 1e-7
    )

    fun initial(): RobotTickI {
        val controllerState = ControllerState()
        var armOffset1 = 0.tick
        var armOffset2 = 0.tick
        if (initialArmPose!=null) {
            val zeros = controllerState.armController.boxTubeKinematics.inverse(DiffyOutputPose(initialArmPose.pivot,initialArmPose.extension))

            armOffset1 = zeros.axis1.cast(tick)
            armOffset2 = zeros.axis2.cast(tick)
        }

        return RobotTickI(
            controllerState,
            t = Clock.milliseconds,
            v = simV,
            turnEncodersPos = List(4) { 0.V },
            armMotor1Pos = armOffset1,
            armMotor2Pos = armOffset2
        )
    }

    override fun update(o: RobotTickO): RobotTickI {
        val dx = { _: Double, x: SimArmState, u: List<Volt> -> this.simArm(x,u) }

        val lastT = o.nextState.lastT
        val nextT = lastT + updateTime.value
        val u = o.armPowers.map { (it*simV).cast(V) }

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
            prefix("sim/arm/dx") {
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

    private val wheelMoment = 1.0 * kg*m*m
    private val wheelRadius = 0.5.m
    val servoDriveMotorModel = goBildaMotorConstants(1.0).constantLoadSystem(wheelMoment)
//    fun simSwerve(x: SimSwerveState, turnUs: List<Volt>, driveUs: List<Volt>, log: Boolean = false): SimSwerveState {
//        for (i in 0..4) {
//            val vDir = x.vel.normalized()
//            val vWheel = x.drives[i].velocity*wheelRadius
//
//            val vLongitudinal = x.vel dot polar(1.m,x.turns[i].position.cast(rad))
//            val slipRatio = (vLongitudinal-vWheel)/vWheel.map { max(it,vLongitudinal.value) }
//            val slipAngle = vDir.theta()-x.turns[i].position.normalizeRadian()
//
//        }
//    }
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

class SimSwerveState(
    val turns: List<MotorState>,
    val drives: List<MotorState>,
    val vel: Vector2,
    val pos: Vector2,
): State<SimSwerveState> {
    override var x: SimSwerveState = this

    override fun toComponents() =
        (turns.map { it.toComponents() } + drives.map { it.toComponents() }).flatten() +
                vel.components.map { it.value } +
                pos.components.map { it.value }

    override fun fromComponents(cs: List<Number>): SimSwerveState {
        val motorStateSize = 3
        val motorStates = cs.chunked(motorStateSize)
        val turns = motorStates.subList(0,4).map { turns[0].fromComponents(it) }
        val drives = motorStates.subList(4,8).map { drives[0].fromComponents(it) }

        return SimSwerveState(turns,drives, vec2(cs[8].m/s,cs[9].m/s), vec2(cs[10].m,cs[11].m))
    }
}