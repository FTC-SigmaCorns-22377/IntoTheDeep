package sigmacorns.common.io

import eu.sirotin.kotunil.base.A
import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.Second
import eu.sirotin.kotunil.base.kg
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.mm
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
import eu.sirotin.kotunil.specialunits.g
import net.unnamedrobotics.lib.math2.Tick
import net.unnamedrobotics.lib.math2.Transform2D
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.clampMagnitude
import net.unnamedrobotics.lib.math2.inches
import net.unnamedrobotics.lib.math2.map
import net.unnamedrobotics.lib.math2.revolution
import net.unnamedrobotics.lib.math2.tick
import net.unnamedrobotics.lib.math2.unitless
import net.unnamedrobotics.lib.physics.LinearTireModel
import net.unnamedrobotics.lib.physics.RK45Integrator
import net.unnamedrobotics.lib.physics.State
import net.unnamedrobotics.lib.physics.SwerveDrivebase
import net.unnamedrobotics.lib.physics.SwerveInput
import net.unnamedrobotics.lib.physics.SwerveState
import net.unnamedrobotics.lib.physics.goBildaMotorConstants
import net.unnamedrobotics.lib.rerun.RerunConnection
import net.unnamedrobotics.lib.rerun.rerun
import sigmacorns.common.Constants
import sigmacorns.common.SimIOTimes
import sigmacorns.common.subsystems.arm.ScoringPose
import sigmacorns.common.subsystems.arm.DiffyKinematics
import sigmacorns.common.subsystems.arm.DiffyOutputPose
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sign
import kotlin.math.sin
import kotlin.random.Random

class SimIO(
    val simV: Volt = 12.V,
    initialPos: Transform2D,
    initialScoringPose: ScoringPose,
    val log: Boolean = false,
    val rerunName: String = "unnamed"
): SigmaIO() {
    override val rerunConnection = RerunConnection(rerunName,"127.0.0.1")

    constructor(
        simV: Volt = 12.V,
        initialPos: Transform2D,
        initialPivot: Radian,
        initialExtension: Metre,
        initialPitch: Radian = 0.rad,
        initialRoll: Radian = 0.rad,
        log: Boolean = false,
        rerunName: String = "unnamed"
    ): this(
        simV,
        initialPos,
        ScoringPose(initialPos.vector(),initialPos.angle.cast(rad),initialExtension,initialPivot,initialPitch,initialRoll),
        log,
        rerunName
    )

    init {
        rerunConnection.setTimeSeconds("sim",0.s)

        val dir = System.getProperty("user.dir")
        rerunConnection.field("$dir/src/test/resources/field_image.png")

        rerun(rerunConnection) {
            transform("field/image", mat = floatArrayOf(0f,1f,0f, 1f,0f,0f, 0f,0f,1f))
        }
    }

    private val boxTubeKinematics = DiffyKinematics(Constants.ARM_PIVOT_RATIO,Constants.ARM_EXTENSION_RATIO)

    private val armIntegrator: RK45Integrator<SimArmState, List<Volt>> = RK45Integrator(
        minStep = 0.001, tolerance = 0.003
    )

    private var armState: SimArmState = SimArmState(initialScoringPose.pivot,initialScoringPose.extension)

    val drivebase = SwerveDrivebase(
        0.048.m,
        18.inches,
        18.inches,
        20.kg,
        31.5.kg* mm * mm *15,
        18.5.kg* mm * mm,
        turnMotor = goBildaMotorConstants(6000.0/435.0),
        driveMotor = goBildaMotorConstants(6000.0/435.0),
        tireModel = LinearTireModel(10.0,1.0,1.0,30.N,30.N,3.N*m),
//        viscousFriction = 0.0
    )

    val swerveIntegrator = drivebase.model.newStateIntegrator(SwerveState(initialPos),RK45Integrator(
        minStep = 0.004, tolerance = 0.005
    ))

    val armReversed = listOf(1,1)

    private val pivotRatio = (Constants.ARM_PULLEY_RATIO.pow(-1) * Constants.ARM_MOTOR_GEAR_RATIO).cast(rad/rad)
    private val extendRatio = ((1.m/2.m) / Constants.ARM_SPOOL_RADIUS / Constants.ARM_DIFFY_RATIO * pivotRatio)
    private fun simArm(x: SimArmState, u: List<Volt>): SimArmState {
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

        val armMoment = x.extension.map { it.absoluteValue } * Constants.ARM_MOMENT_RATIO + 20000.g*mm*mm

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

    var simTime: Second = 0.s
    fun stepSim(dt: Second) {
        fun actuatorToPower(actuator: Actuator<Double>) =
            ((actuator as SimActuator).u
                ?.clampMagnitude(1.0)?.takeIf { !it.isNaN() }
                ?: 0.0).unitless()

        armState = armIntegrator.integrate(
            { _,x,u -> simArm(x, u) },
            simTime.value,(simTime+dt).value,armState,
            armMotorPowers.zip(armReversed).map { (actuatorToPower(it.first)*simV*it.second).cast(V) }
        ).second.last()

        swerveIntegrator.integrate(dt, SwerveInput(
            turnPowers.map { actuatorToPower(it) },
            drivePowers.map { actuatorToPower(it) },
        ))

        simTime = (simTime+dt).cast(s)
    }

    override fun clearBulkCache() = stepSim(randomizeIOTime(SimIOTimes.bulkRead))

    /* IO OBJECTS */

    private val armPosSensor = sensor(bulkReadable = true, name = "armPos") {
        val armPos = boxTubeKinematics.inverse(DiffyOutputPose(armState.pivot,armState.extension))
        listOf(armPos.axis1, armPos.axis2).zip(armReversed).map { (it.first*it.second).cast(tick) }
    }

    context(ControlLoopContext<*, *, *, SigmaIO,*>) override fun armPositions(): List<Tick> = armPosSensor.get()

    private val turnVoltageSensor = sensor(bulkReadable = true, name = "turnVoltage") {
        swerveIntegrator.state.turns.zip(Constants.MODULE_OFFSET).map {
            (((it.first.position + it.second)/revolution)*3.3.V).map { it.mod(3.3.V.value) }.cast(V)
        }
    }

    context(ControlLoopContext<*, *, *, SigmaIO,*>)
    override fun turnVoltages(): List<Volt> = turnVoltageSensor.get()

    private val posSensor = sensor(name = "pos") {
        stepSim(randomizeIOTime(SimIOTimes.pinpointFetch))
        swerveIntegrator.state.pos
    }

    private val velSensor = sensor(name = "vel") {
        stepSim(randomizeIOTime(SimIOTimes.pinpointFetch))
        swerveIntegrator.state.vel
    }

    context(ControlLoopContext<*, *, *, SigmaIO,*>) override fun position() = posSensor.get()
    context(ControlLoopContext<*, *, *, SigmaIO,*>) override fun velocity() = velSensor.get()

    fun randomizeIOTime(t: Second): Second
        = t.map { max(it + it*Random.nextDouble(-SimIOTimes.uncertainty,SimIOTimes.uncertainty),0.5.ms.value) }

    private class SimActuator<T : Any>(val stepTime: Expression? = null, val io: SimIO): Actuator<T> {
        var u: T? = null
        override fun write(u: T) {
            this.u = u
            if(stepTime != null) io.stepSim(io.randomizeIOTime(stepTime.cast(s)))
        }
    }

    override val armMotorPowers: List<Actuator<Double>> = List(2) { SimActuator(SimIOTimes.motorWrite,this) }
    override val drivePowers: List<Actuator<Double>> = List(4) { SimActuator(SimIOTimes.motorWrite,this) }
    override val turnPowers: List<Actuator<Double>> = List(4) { SimActuator(SimIOTimes.servoWrite,this) }
    override val clawPos: Actuator<Double> = SimActuator(SimIOTimes.servoWrite,this)
    override val diffyPos: List<Actuator<Double>> = List(2) { SimActuator(SimIOTimes.servoWrite,this) }

    override fun time(): Second {
        simTime = (simTime + SimIOTimes.base).cast(s)
        rerunConnection.setTimeSeconds("sim",simTime)
        return simTime
    }

    override fun voltage() = simV
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