package fr.radi3nt.physics.dynamics.island;

import fr.radi3nt.physics.core.state.RigidBody;

public class ArrayRigidBodyIsland implements RigidBodyIsland, EditableRigidBodyIsland {

    protected RigidBody[] rigidBodies;

    public void setRigidBody(RigidBody rigidBody, int index) {
        rigidBodies[index] = rigidBody;
    }

    public void setSize(int size) {
        rigidBodies = new RigidBody[size];
    }

    @Override
    public void copyStates(RigidBodyIsland island) {
        for (int i = 0; i < rigidBodies.length; i++) {
            RigidBody rigidBody = rigidBodies[i];
            RigidBody copiedFrom = island.getRigidBody(i);
            rigidBody.setState(copiedFrom.getDynamicsData());
        }
    }

    @Override
    public RigidBody getRigidBody(int index) {
        return rigidBodies[index];
    }

    @Override
    public int getSize() {
        return rigidBodies.length;
    }
}