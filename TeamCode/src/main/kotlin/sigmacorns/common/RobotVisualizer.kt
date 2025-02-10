package sigmacorns.common

import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.Radian
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math.RGBA
import net.unnamedrobotics.lib.math2.Twist2D
import net.unnamedrobotics.lib.math2.Vector3
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.inverseLerp
import net.unnamedrobotics.lib.math2.map
import net.unnamedrobotics.lib.math2.mapRanges
import net.unnamedrobotics.lib.math2.rotateY
import net.unnamedrobotics.lib.math2.spherical
import net.unnamedrobotics.lib.math2.vec3
import net.unnamedrobotics.lib.math2.withZ
import net.unnamedrobotics.lib.math2.z
import net.unnamedrobotics.lib.rerun.RerunConnection
import net.unnamedrobotics.lib.rerun.RerunPrefix
import net.unnamedrobotics.lib.rerun.archetypes.Boxes3D
import net.unnamedrobotics.lib.rerun.archetypes.Points3D
import net.unnamedrobotics.lib.rerun.rerun
import org.joml.Quaterniond
import sigmacorns.common.io.SigmaIO
import sigmacorns.common.kinematics.IntakeAngleKinematics
import sigmacorns.constants.Limits
import sigmacorns.constants.Physical
import sigmacorns.constants.Physical.EXTEND_M_PER_TICK
import sigmacorns.constants.Physical.LIFT_M_PER_TICK
import sigmacorns.constants.Tuning
import sigmacorns.constants.Visualization
import kotlin.math.absoluteValue

class RobotVisualizer(
    val io: SigmaIO
) {
    fun init() {
        rerun(io.rerunConnection) {
            setTimeSeconds("sim",io.time())
            prefix("robot") {
                log("drivebase") {
                    Boxes3D(
                        halfSizes = listOf(Visualization.DRIVEBASE_SIZE/2.0),
                        centers = listOf(vec3(0.0,0.0,Visualization.DRIVEBASE_SIZE.z.value/2.0))
                    )
                }
            }
        }
    }

    fun log() {
        rerun(io.rerunConnection) {
            setTimeSeconds("sim",io.time())
            logScalars()
            logTwist(io.velocity(), "vel", 10)
            logSlides()
            transform("robot", io.position().vector().withZ(0.m))
        }
    }

    context(RerunPrefix, RerunConnection)
    private fun logSlides() {
        val m1 = io.motor1Pos()
        val m2 = io.motor2Pos()
        val extension = EXTEND_M_PER_TICK*(m1+m2)/2.0
        val lift = LIFT_M_PER_TICK*(m1-m2)/2.0

        scalar("extension(m)",extension.value)
        scalar("lift(m)",lift.value)

        logSlides("intakeSlidesL",Visualization.SLIDE_SIZE,Visualization.INTAKE_LEFT_END_POS,0.rad,90.degrees,extension.cast(m),Visualization.INTAKE_STAGES)
        logSlides("intakeSlidesR",Visualization.SLIDE_SIZE,Visualization.INTAKE_RIGHT_END_POS,0.rad,90.degrees,extension.cast(m),Visualization.INTAKE_STAGES)

        logSlides("liftSlidesL",Visualization.SLIDE_SIZE,Visualization.LIFT_LEFT_END_POS,0.rad,0.rad,lift.cast(m),Visualization.LIFT_STAGES)
        logSlides("liftSlidesR",Visualization.SLIDE_SIZE,Visualization.LIFT_RIGHT_END_POS,0.rad,0.rad,lift.cast(m),Visualization.LIFT_STAGES)

        val arm1 = mapRanges(0.0..1.0,Limits.ARM_SERVO_1.let { it.min.value..it.max.value })(io.armL).rad
        val arm2 = mapRanges(0.0..1.0,Limits.ARM_SERVO_2.let { it.min.value..it.max.value })(io.armR).rad
        val arm = (arm1+arm2)/2.0
        val wrist =  arm - (arm1-arm2)/2.0

        val armStart = (Visualization.LIFT_LEFT_END_POS + spherical(lift, 0.rad, 0.rad)).let {
            vec3(it.x, 0.m, it.z-Visualization.ARM_OFFSET)
        }
        val armEnd = armStart + spherical(Physical.ARM_LENGTH,0.rad,arm.cast(rad))
        val clawEnd = armEnd + spherical(Physical.CLAW_LENGTH,0.rad,wrist.cast(rad))

        logRectP2P("arm",armStart,armEnd,Visualization.ARM_HEIGHT,Visualization.ARM_WIDTH)

        val clawClosedT = (Tuning.CLAW_OPEN..Tuning.CLAW_OPEN).inverseLerp(io.claw)
        logRectP2P("claw",armEnd,clawEnd,Visualization.ARM_HEIGHT,Visualization.ARM_WIDTH, color = RGBA((1-clawClosedT),clawClosedT,0.0,0.0))

        val intakeEnd = (Visualization.INTAKE_LEFT_END_POS + spherical(extension, 0.rad, 90.degrees)).let {
            vec3(it.x, 0.m, it.z)
        }

        val intakeServoPos = mapRanges(0.0..1.0, Limits.INTAKE_SERVO_1.let { it.min.value..it.max.value })(io.intakeL).rad
        val intakePos = IntakeAngleKinematics.forward(intakeServoPos)

//        prefix("kinematics") {
//            log("circle1") {
//                Ellipsoids3D(
//                    centers = listOf(IntakeAngleKinematics.lastCircle1Pos + intakeEnd),
//                    sizes = listOf(vec3(IntakeAngleKinematics.lastCircle1Radius,0.m,IntakeAngleKinematics.lastCircle1Radius))
//                )
//            }
//
//            log("circle2") {
//                Ellipsoids3D(
//                    centers = listOf(IntakeAngleKinematics.lastCircle2Pos + intakeEnd),
//                    sizes = listOf(vec3(IntakeAngleKinematics.lastCircle2Radius,0.m,IntakeAngleKinematics.lastCircle2Radius))
//                )
//            }
//
//            log("intersection") {
//                Points3D(origins = listOf(IntakeAngleKinematics.lastIntersectionPos + intakeEnd))
//            }
//        }

//        println("intakePos = $intakePos")

        prefix("links") {
            val linkServoPos = Physical.INTAKE_SERVO_POS + intakeEnd
            val link1End = linkServoPos + spherical(Physical.INTAKE_LINKAGE_1_LEN,0.rad,intakeServoPos)
                .normalized()*Physical.INTAKE_LINKAGE_1_LEN
            val link2End = (Physical.INTAKE_LINKAGE_AXLE_POS + intakeEnd + Physical.INTAKE_LINKAGE_END_POS.rotateY(intakePos.map { -it })).let {
                link1End + (it-link1End).normalized()*Physical.INTAKE_LINKAGE_2_LEN
            }

            logRectP2P("intakeLink1",
                linkServoPos,
                link1End,
                Visualization.INTAKE_LINK_WIDTH,
                Visualization.INTAKE_LINK_WIDTH,
            )

            logRectP2P("intakeLink2",
                link1End,
                link2End,
                Visualization.INTAKE_LINK_WIDTH,
                Visualization.INTAKE_LINK_WIDTH,
            )
        }


        logRectP2P("intake",
            IntakeAngleKinematics.offsetFromSlideEnd(intakePos) + intakeEnd,
            IntakeAngleKinematics.offsetFromSlideEnd(intakePos,(-Visualization.INTAKE_HEIGHT).cast(m)) + intakeEnd,
            Visualization.INTAKE_LENGTH,Visualization.INTAKE_WIDTH
        )
    }

    context(RerunPrefix, RerunConnection)
    private fun logScalars() {
        prefix("sensors") {
            scalar("x", io.position().x.value)
            scalar("y", io.position().y.value)
            scalar("θ", io.position().angle.value)

            scalar("vx", io.velocity().dx.value)
            scalar("vy", io.velocity().dy.value)
            scalar("vθ", io.velocity().dAngle.value)

            scalar("motor1", io.motor1Pos().value)
            scalar("motor2", io.motor2Pos().value)
        }

        prefix("actuators") {
            scalar("FL",io.driveFL)
            scalar("BL",io.driveBL)
            scalar("BR",io.driveBR)
            scalar("FR",io.driveFR)

            scalar("motor1",io.motor1)
            scalar("motor2",io.motor2)

            scalar("intake",io.intake)
            scalar("intakeL",io.intakeL)
            scalar("intakeR",io.intakeR)

            scalar("armL",io.armL)
            scalar("armR",io.armR)
            scalar("claw",io.claw)

            scalar("tiltL",io.tilt1)
            scalar("tiltR",io.tilt2)
        }
    }

    context(RerunPrefix, RerunConnection)
    private fun logTwist(twist: Twist2D, name: String, samples: Int, length: Number = 1.0) {
        log(name) {
            Points3D(
                origins = (0..samples)
                    .map {
                        (twist * length*(it.toDouble()/samples.toDouble())).exp().vector().withZ(0.m)
                    },
                radii = List(samples) { 0.0 }
            )
        }
    }

    context(RerunPrefix, RerunConnection)
    private fun logSlides(
        name: String,
        slideSize: Vector3,
        zeroEndPos: Vector3,
        theta: Radian,
        phi: Radian,
        extension: Metre,
        stages: Int
    ) {
        val halfSize = slideSize/2.0
        val positions = (0..stages)
            .map { (it.toDouble()/stages.toDouble()) }
            .map { zeroEndPos + spherical(extension*it - halfSize.x,theta,phi) }
        log(name) {
            Boxes3D(
                halfSizes = List(stages + 1) { halfSize },
                centers = positions,
                rotations = List(stages + 1) { Quaterniond().rotateY((90.degrees-phi).value).rotateZ(theta.value) }
            )
        }
    }

    context(RerunPrefix, RerunConnection)
    private fun logRectP2P(
        name: String,
        p1: Vector3,
        p2: Vector3,
        height: Metre,
        width: Metre,
        roll: Radian = 0.rad,
        color: RGBA? = null
    ) {
        val center = (p1+p2)/2.0
        val v = p1-p2
        val halfLength = v.magnitude()/2.0
        val theta = v.theta()
        val phi = v.phi().let { if(theta.map { it.absoluteValue } > 90.degrees) it else -it }

        val rotation = Quaterniond().rotateY((90.degrees-phi).value).rotateZ(theta.value)

        log(name) {
            Boxes3D(
                listOf(vec3(halfLength,width/2.0,height/2.0)),
                listOf(center),
                rotations = listOf(rotation),
                colors = color?.let { listOf(it) }
            )
        }
    }
}