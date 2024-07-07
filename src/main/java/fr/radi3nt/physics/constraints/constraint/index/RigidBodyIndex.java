package fr.radi3nt.physics.constraints.constraint.index;

import fr.radi3nt.physics.core.state.DynamicsData;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

public interface RigidBodyIndex {

    DynamicsData getData(RigidBodyIsland island);
    RigidBody getRigidBody(RigidBodyIsland island);
    IdentifiedDynamicsData getIdentifiedData(RigidBodyIsland island);
    int getRigidBodyId();

}
