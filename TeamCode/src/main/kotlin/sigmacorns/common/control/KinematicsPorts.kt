package sigmacorns.common.control

import net.unnamedrobotics.lib.control.circuit.ControlCircuitBuilder
import net.unnamedrobotics.lib.control.circuit.PortOut
import net.unnamedrobotics.lib.control.circuit.λPortKind
import net.unnamedrobotics.lib.physics.Kinematics

context(ControlCircuitBuilder)
infix fun <I,O> PortOut<I,*>.connect(rhs: Kinematics<I,O>): PortOut<O, λPortKind> {
    return (this connect (rhs::forward))
}