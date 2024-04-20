package fr.radi3nt.physics.dynamics.island;

import fr.radi3nt.physics.core.state.RigidBody;

public interface RigidBodyIsland {

    void copyStates(RigidBodyIsland island);

    RigidBody getRigidBody(int index);
    int getSize();
}
