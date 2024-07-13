package fr.radi3nt.physics.dynamics.island;

import java.util.ArrayList;

public class ArrayListRigidBodyIsland extends ListRigidBodyIsland {

    public ArrayListRigidBodyIsland() {
        super(new ArrayList<>());
    }

    public void clear() {
        rigidBodies.clear();
    }
}
