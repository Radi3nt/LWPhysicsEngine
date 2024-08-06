package fr.radi3nt.physics.dynamics.ode.integrator;

import fr.radi3nt.physics.dynamics.force.accumulator.MotionAccumulator;
import fr.radi3nt.physics.dynamics.island.EditableRigidBodyIsland;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

public interface Integrator {

    void integrate(RigidBodyIsland original, EditableRigidBodyIsland result, MotionAccumulator forces, float dt);

}
