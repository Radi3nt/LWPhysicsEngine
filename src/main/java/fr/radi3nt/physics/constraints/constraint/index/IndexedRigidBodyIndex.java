package fr.radi3nt.physics.constraints.constraint.index;

import fr.radi3nt.physics.core.state.DynamicsData;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

public class IndexedRigidBodyIndex implements RigidBodyIndex {

    private final int bodyId;

    public IndexedRigidBodyIndex(int bodyId) {
        this.bodyId = bodyId;
    }

    @Override
    public DynamicsData getData(RigidBodyIsland island) {
        return getRigidBody(island).getDynamicsData();
    }

    @Override
    public RigidBody getRigidBody(RigidBodyIsland island) {
        return island.getRigidBodyById(bodyId);
    }

    @Override
    public int getRigidBodyId() {
        return bodyId;
    }

    @Override
    public IdentifiedDynamicsData getIdentifiedData(RigidBodyIsland island) {
        RigidBody rigidBody = getRigidBody(island);
        if (rigidBody==null)
            return null;
        return new IdentifiedDynamicsData(rigidBody.getDynamicsData(), rigidBody.getSleepingData(), rigidBody.getRigidBodyId());
    }

    @Override
    public String toString() {
        return "IndexedRigidBodyIndex{" +
                "islandIndex=" + bodyId +
                '}';
    }
}
