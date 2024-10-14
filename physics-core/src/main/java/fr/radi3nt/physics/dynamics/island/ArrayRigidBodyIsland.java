package fr.radi3nt.physics.dynamics.island;

import fr.radi3nt.physics.core.state.RigidBody;

import java.util.Arrays;
import java.util.Collections;
import java.util.Iterator;

public class ArrayRigidBodyIsland implements RigidBodyIsland, EditableRigidBodyIsland {

    protected RigidBody[] rigidBodies;

    public ArrayRigidBodyIsland() {
    }

    public ArrayRigidBodyIsland(RigidBody... rigidBodies) {
        this.rigidBodies = rigidBodies;
    }

    public void setRigidBody(RigidBody rigidBody, int index) {
        rigidBodies[index] = rigidBody;
    }

    public void copy(RigidBodyIsland island) {
        setSize(island.getSize());
        for (int i = 0; i < rigidBodies.length; i++) {
            rigidBodies[i] = island.getRigidBody(i);
        }
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
    public RigidBody getRigidBodyById(int id) {
        for (RigidBody rigidBody : rigidBodies) {
            if (rigidBody.getRigidBodyId()==id)
                return rigidBody;
        }
        return null;
    }

    @Override
    public int getRigidBodyIndexById(int id) {
        for (int i = 0, rigidBodiesLength = rigidBodies.length; i < rigidBodiesLength; i++) {
            RigidBody rigidBody = rigidBodies[i];
            if (rigidBody.getRigidBodyId() == id)
                return i;
        }
        return -1;
    }

    @Override
    public int getSize() {
        return rigidBodies.length;
    }

    @Override
    public Iterator<RigidBody> iterator() {
        return Collections.unmodifiableCollection(Arrays.asList(rigidBodies)).iterator();
    }
}