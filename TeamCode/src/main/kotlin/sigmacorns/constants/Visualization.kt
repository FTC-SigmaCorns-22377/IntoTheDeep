package sigmacorns.constants

import eu.sirotin.kotunil.base.Metre
import eu.sirotin.kotunil.base.cm
import eu.sirotin.kotunil.base.m
import eu.sirotin.kotunil.base.mm
import eu.sirotin.kotunil.core.*
import net.unnamedrobotics.lib.math2.cast
import net.unnamedrobotics.lib.math2.degrees
import net.unnamedrobotics.lib.math2.vec3
import net.unnamedrobotics.lib.math2.z

object Visualization {
    val DRIVEBASE_SIZE = vec3(336.mm,276.7.mm,116.mm)
    val SLIDE_SIZE = vec3(240.mm,10.mm,10.mm)

    val INTAKE_LEFT_END_POS = vec3(DRIVEBASE_SIZE.x/2.0, DRIVEBASE_SIZE.y/2.0, DRIVEBASE_SIZE.z/2.0)
    val INTAKE_RIGHT_END_POS = vec3(DRIVEBASE_SIZE.x/2.0,-DRIVEBASE_SIZE.y/2.0,DRIVEBASE_SIZE.z/2.0)
    val INTAKE_STAGES = 4

    val LIFT_LEFT_END_POS = vec3(0.mm,DRIVEBASE_SIZE.y/2.0,SLIDE_SIZE.x)
    val LIFT_RIGHT_END_POS = vec3(0.mm,-DRIVEBASE_SIZE.y/2.0,SLIDE_SIZE.x)
    val LIFT_STAGES = 4

    val ARM_OFFSET = 20.mm
    val ARM_WIDTH: Metre = DRIVEBASE_SIZE.y.cast(m)
    val ARM_HEIGHT: Metre = 5.cm

    val INTAKE_LENGTH = 5.cm
    val INTAKE_HEIGHT = 133.11078.mm
    val INTAKE_WIDTH = 5.cm

    val INTAKE_LINK_WIDTH = 2.cm

    val CLAW_OFFSET = 3.cm
    val CLAW_LENGTH = 6.cm
    val CLAW_WIDTH = 1.cm
    val CLAW_CLOSE_ANGLE = 0.degrees
    val CLAW_OPEN_ANGLE = 60.degrees
}