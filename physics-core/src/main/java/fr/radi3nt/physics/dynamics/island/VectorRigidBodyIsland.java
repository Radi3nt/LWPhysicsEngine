package fr.radi3nt.physics.dynamics.island;

import fr.radi3nt.physics.core.state.RigidBody;

import java.util.Vector;

public class VectorRigidBodyIsland extends ListRigidBodyIsland {

    public VectorRigidBodyIsland() {
        super(new Vector<>());
    }

    @Override
    public void add(RigidBody rigidBody) {
        setSize(Math.max(rigidBodies.size(), rigidBody.getRigidBodyId()));
        rigidBodies.add(rigidBody.getRigidBodyId(), rigidBody);
    }

    public void setSize(int size) {
        ((Vector<RigidBody>) rigidBodies).setSize(size);
    }
}
