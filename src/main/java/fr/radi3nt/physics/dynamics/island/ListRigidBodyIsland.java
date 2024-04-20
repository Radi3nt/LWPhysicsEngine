package fr.radi3nt.physics.dynamics.island;

import fr.radi3nt.physics.core.state.RigidBody;

import java.util.List;
import java.util.Vector;

public class ListRigidBodyIsland implements RigidBodyIsland, EditableRigidBodyIsland {

    protected final List<RigidBody> rigidBodies;

    public ListRigidBodyIsland(List<RigidBody> rigidBodies) {
        this.rigidBodies = rigidBodies;
    }

    public void add(RigidBody rigidBody) {
        rigidBodies.add(rigidBody);
    }

    public void remove(RigidBody rigidBody) {
        rigidBodies.remove(rigidBody);
    }

    public void setRigidBody(RigidBody rigidBody, int index) {
        rigidBodies.set(index, rigidBody);
    }

    @Override
    public void copyStates(RigidBodyIsland island) {
        for (int i = 0; i < rigidBodies.size(); i++) {
            RigidBody rigidBody = rigidBodies.get(i);
            RigidBody copiedFrom = island.getRigidBody(i);
            rigidBody.setState(copiedFrom.getDynamicsData());
        }
    }

    @Override
    public RigidBody getRigidBody(int index) {
        return rigidBodies.get(index);
    }

    @Override
    public int getSize() {
        return rigidBodies.size();
    }
}
