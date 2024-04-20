package fr.radi3nt.physics.dynamics.island;

import fr.radi3nt.physics.core.state.RigidBody;

public interface EditableRigidBodyIsland extends RigidBodyIsland {

    void setRigidBody(RigidBody rigidBody, int index);

}
