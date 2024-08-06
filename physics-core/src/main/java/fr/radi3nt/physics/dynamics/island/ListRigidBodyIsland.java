package fr.radi3nt.physics.dynamics.island;

import fr.radi3nt.physics.core.state.RigidBody;

import java.util.Collections;
import java.util.Iterator;
import java.util.List;

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
    public RigidBody getRigidBodyById(int id) {
        for (RigidBody rigidBody : rigidBodies) {
            if (rigidBody.getRigidBodyId()==id)
                return rigidBody;
        }
        return null;
    }

    @Override
    public int getRigidBodyIndexById(int id) {
        for (int i = 0, rigidBodiesSize = rigidBodies.size(); i < rigidBodiesSize; i++) {
            RigidBody rigidBody = rigidBodies.get(i);
            if (rigidBody.getRigidBodyId() == id)
                return i;
        }
        return -1;
    }

    @Override
    public int getSize() {
        return rigidBodies.size();
    }

    @Override
    public Iterator<RigidBody> iterator() {
        return Collections.unmodifiableCollection(rigidBodies).iterator();
    }
}
