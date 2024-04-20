package fr.radi3nt.physics.constraints.constraint.index;

import fr.radi3nt.physics.core.state.DynamicsData;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

public class IndexedRigidBodyIndex implements RigidBodyIndex {

    private final int islandIndex;

    public IndexedRigidBodyIndex(int islandIndex) {
        this.islandIndex = islandIndex;
    }

    @Override
    public DynamicsData getData(RigidBodyIsland island) {
        return getRigidBody(island).getDynamicsData();
    }

    @Override
    public RigidBody getRigidBody(RigidBodyIsland island) {
        return island.getRigidBody(islandIndex);
    }

    @Override
    public int getRigidBodyId(RigidBodyIsland island) {
        return getRigidBody(island).getRigidBodyId();
    }

    @Override
    public IdentifiedDynamicsData getIdentifiedData(RigidBodyIsland island) {
        RigidBody rigidBody = getRigidBody(island);
        return new IdentifiedDynamicsData(rigidBody.getDynamicsData(), rigidBody.getSleepingData(), rigidBody.getRigidBodyId());
    }
}
