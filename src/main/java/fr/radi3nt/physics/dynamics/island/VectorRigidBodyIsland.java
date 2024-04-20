package fr.radi3nt.physics.dynamics.island;

import fr.radi3nt.physics.core.state.RigidBody;

import java.util.List;
import java.util.Vector;

public class VectorRigidBodyIsland extends ListRigidBodyIsland {

    public VectorRigidBodyIsland() {
        super(new Vector<>());
    }

    public void setSize(int size) {
        ((Vector<RigidBody>) rigidBodies).setSize(size);
    }
}
