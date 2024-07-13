package fr.radi3nt.physics.dynamics.force.caster;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.force.accumulator.ForceAccumulator;
import fr.radi3nt.physics.dynamics.force.accumulator.ForceResult;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

public class MassedVectorForceCaster implements ForceCaster {

    private final ForceResult cachingForceResult = new ForceResult();
    private final Vector3f force;
    private final Vector3f torque;

    public MassedVectorForceCaster(Vector3f force, Vector3f torque) {
        this.force = force;
        this.torque = torque;
    }

    @Override
    public void cast(ForceAccumulator accumulator, RigidBodyIsland island, float dt) {
        for (int i = 0; i < island.getSize(); i++) {
            RigidBody rigidBody = island.getRigidBody(i);
            Vector3f currentForce = force.duplicate();
            float inverseMass = rigidBody.getDynamicsData().getBodyProperties().inverseMass;
            if (inverseMass>0)
                currentForce.mul(1/inverseMass);
            cachingForceResult.set(currentForce, torque);
            accumulator.addForce(cachingForceResult, i);
        }
    }
}
