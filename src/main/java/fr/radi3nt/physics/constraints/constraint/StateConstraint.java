package fr.radi3nt.physics.constraints.constraint;

import fr.radi3nt.maths.components.arbitrary.VectorNf;
import fr.radi3nt.physics.constraints.constraint.index.IdentifiedDynamicsData;
import fr.radi3nt.physics.constraints.constraint.index.RigidBodyIndex;

public class StateConstraint {

    public static final int STATE_STRIDE = 6;

    private final IdentifiedDynamicsData[] concernedBodies;
    private final VectorNf[] states;

    public StateConstraint(IdentifiedDynamicsData[] concernedBodies, VectorNf[] states) {
        this.concernedBodies = concernedBodies;
        this.states = states;
    }

    public IdentifiedDynamicsData[] getConcernedBodies() {
        return concernedBodies;
    }

    public VectorNf[] getStates() {
        return states;
    }
}
