package fr.radi3nt.physics.dynamics.island;

import fr.radi3nt.physics.core.state.RigidBody;

import java.util.Iterator;

public interface RigidBodyIsland {

    void copyStates(RigidBodyIsland island);

    RigidBody getRigidBody(int index);
    RigidBody getRigidBodyById(int id);
    int getRigidBodyIndexById(int id);
    int getSize();

    Iterator<RigidBody> iterator();
}
