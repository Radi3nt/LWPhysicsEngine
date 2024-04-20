package fr.radi3nt.physics.constraints.constraint.index;

import fr.radi3nt.physics.core.state.DynamicsData;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

public class ImplicitRigidBodyIndex implements RigidBodyIndex {

    private final RigidBody rigidBody;

    public ImplicitRigidBodyIndex(RigidBody rigidBody) {
        this.rigidBody = rigidBody;
    }

    @Override
    public DynamicsData getData(RigidBodyIsland island) {
        return rigidBody.getDynamicsData();
    }

    @Override
    public RigidBody getRigidBody(RigidBodyIsland island) {
        return rigidBody;
    }

    @Override
    public int getRigidBodyId(RigidBodyIsland island) {
        return rigidBody.getRigidBodyId();
    }

    @Override
    public IdentifiedDynamicsData getIdentifiedData(RigidBodyIsland island) {
        return new IdentifiedDynamicsData(rigidBody.getDynamicsData(), rigidBody.getSleepingData(), rigidBody.getRigidBodyId());
    }
}
