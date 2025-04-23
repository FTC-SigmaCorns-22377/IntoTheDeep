package sigmacorns.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import sigmacorns.constants.SampleColors

@TeleOp
class AABlueTele: Teleop() {
    override var acceptableColors: Set<SampleColors> = setOf(SampleColors.YELLOW,SampleColors.BLUE)
}

@TeleOp
class AARedTele: Teleop() {
    override var acceptableColors: Set<SampleColors> = setOf(SampleColors.YELLOW,SampleColors.RED)
}