package fr.radi3nt.physics.dynamics.ode.integrator;

import fr.radi3nt.physics.dynamics.island.EditableRigidBodyIsland;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;
import fr.radi3nt.physics.dynamics.force.accumulator.ForceAccumulator;

public interface Integrator {

    void integrate(RigidBodyIsland original, EditableRigidBodyIsland result, ForceAccumulator forces, float dt);

}
