package sigmacorns.common.kinematics

import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.core.*
import eu.sirotin.kotunil.derived.Radian
import eu.sirotin.kotunil.derived.rad
import net.unnamedrobotics.lib.math2.Vector3
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.map
import net.unnamedrobotics.lib.math2.rotateY
import net.unnamedrobotics.lib.math2.spherical
import net.unnamedrobotics.lib.math2.vec3
import net.unnamedrobotics.lib.math2.z
import net.unnamedrobotics.lib.physics.Kinematics
import sigmacorns.constants.Physical
import kotlin.math.absoluteValue

object IntakeAngleKinematics: Kinematics<Radian,Radian> {
    override fun forward(x: Radian): Radian {
        val circle1Center = vec3(0.m,0.m,0.m)
        val circle2Center = Physical.INTAKE_SERVO_POS + spherical(Physical.INTAKE_LINKAGE_1_LEN,0.rad,x) - Physical.INTAKE_LINKAGE_AXLE_POS
        val circle1Radius = Physical.INTAKE_LINKAGE_END_POS.magnitude()
        val circle2Radius = Physical.INTAKE_LINKAGE_2_LEN

        val x1 = circle1Center.x.value
        val x2 = circle2Center.x.value
        val y1 = circle1Center.z.value
        val y2 = circle2Center.z.value
        val r1 = circle1Radius.value
        val r2 = circle2Radius.value

        val ps = intersectTwoCircles(x1,y1,r1,x2,y2,r2)

        val p = if(ps.size==2) {
            val p1 = ps[0].let { vec3(it.first.m, 0.m, it.second.m) }
            val p2 = ps[1].let { vec3(it.first.m, 0.m, it.second.m) }

            //take the intersection that is further away from straight up.
            if(p1.phi()>p2.phi()) p1 else p2
        } else {

            //closest point to the 2 circles
            val t = circle1Radius/(circle1Radius+circle2Radius)
            circle1Center*t + circle2Center*(1.0-t.value)
        }


        val straightP = Physical.INTAKE_LINKAGE_END_POS-Physical.INTAKE_LINKAGE_AXLE_POS

        val angle = p.phi() - straightP.phi()

        return angle.cast(rad)
    }

    lateinit var lastCircle1Pos: Vector3
    lateinit var lastCircle2Pos: Vector3
    lateinit var lastIntersectionPos: Vector3
    var lastCircle1Radius = Physical.INTAKE_LINKAGE_2_LEN
    var lastCircle2Radius = Physical.INTAKE_LINKAGE_1_LEN

    override fun inverse(x: Radian): Radian {
        val circle1Center = Physical.INTAKE_LINKAGE_END_POS.rotateY(x.map { -it })
        val circle1Radius = Physical.INTAKE_LINKAGE_2_LEN
        val circle2Center = Physical.INTAKE_SERVO_POS-Physical.INTAKE_LINKAGE_AXLE_POS
        val circle2Radius = Physical.INTAKE_LINKAGE_1_LEN

        val x1 = circle1Center.x.value
        val x2 = circle2Center.x.value
        val y1 = circle1Center.z.value
        val y2 = circle2Center.z.value
        val r1 = circle1Radius.value
        val r2 = circle2Radius.value

        val ps = intersectTwoCircles(x1,y1,r1,x2,y2,r2)

        val p = if(ps.size==2) {
            val p1 = ps[0].let { vec3(it.first.m,0.m,it.second.m) }
            val p2 = ps[1].let { vec3(it.first.m,0.m,it.second.m) }

            //take the intersection that has the least phi
            if(p1.phi()<p2.phi()) p1 else p2
        } else {
            //closest point to the 2 circles
            val t = circle1Radius/(circle1Radius+circle2Radius)
            circle1Center*t + circle2Center*(1.0-t.value)
        }

        val angle = (p-circle2Center).let {
            (if(it.theta().map { it.absoluteValue } > 90.degrees) -1.0 else 1.0)*it.phi()
        }

        lastCircle1Pos = circle1Center + Physical.INTAKE_LINKAGE_AXLE_POS
        lastCircle2Pos = circle2Center + Physical.INTAKE_LINKAGE_AXLE_POS
        lastIntersectionPos = p + Physical.INTAKE_LINKAGE_AXLE_POS

        return angle.cast(rad)
    }

    /**
     * @param angle The angle of the intake
     * @param dist The distance along the path of the sample where the offset is measured from.
     * When dist=0, this will output the position of the end of a sample that is just at the opening of the top of the intake.
     */
    fun offsetFromSlideEnd(angle: Radian, dist: Metre = 0.m): Vector3
        = Physical.INTAKE_LINKAGE_AXLE_POS + Physical.INTAKE_CENTER_POS.rotateY(angle.map { -it }) + spherical(dist,0.rad,angle)
}

// from https://gist.github.com/jupdike/bfe5eb23d1c395d8a0a1a4ddd94882ac
// x1,y1 is the center of the first circle, with radius r1
// x2,y2 is the center of the second ricle, with radius r2
private fun intersectTwoCircles( x1: Double,y1: Double,r1: Double, x2: Double,y2: Double,r2: Double): List<Pair<Double, Double>> {
    val centerdx = x1 - x2;
    val centerdy = y1 - y2;
    val R = Math.sqrt(centerdx * centerdx + centerdy * centerdy);
    if (!(Math.abs(r1 - r2) <= R && R <= r1 + r2)) { // no intersection
        return emptyList(); // empty list of results
    }
    // intersection(s) should exist

    val R2 = R*R;
    val R4 = R2*R2;
    val a = (r1*r1 - r2*r2) / (2 * R2);
    val r2r2 = (r1*r1 - r2*r2);
    val c = Math.sqrt(2 * (r1*r1 + r2*r2) / R2 - (r2r2 * r2r2) / R4 - 1);

    val fx = (x1+x2) / 2 + a * (x2 - x1);
    val gx = c * (y2 - y1) / 2;
    val ix1 = fx + gx;
    val ix2 = fx - gx;

    val fy = (y1+y2) / 2 + a * (y2 - y1);
    val gy = c * (x1 - x2) / 2;
    val iy1 = fy + gy;
    val iy2 = fy - gy;

    // note if gy == 0 and gx == 0 then the circles are tangent and there is only one solution
    // but that one solution will just be duplicated as the code is currently written
    return listOf(ix1 to iy1, ix2 to iy2)
}

//object IntakeKinematics: Kinematics<> {
//
//}