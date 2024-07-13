package fr.radi3nt.physics.dynamics.force.caster;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.force.accumulator.ForceAccumulator;
import fr.radi3nt.physics.dynamics.force.accumulator.ForceResult;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

public class AttractionForceCaster implements ForceCaster {

    private final ForceResult cachingForceResult = new ForceResult();
    private final float g;
    private final Vector3f torque;

    public AttractionForceCaster(float g, Vector3f torque) {
        this.g = g;
        this.torque = torque;
    }

    @Override
    public void cast(ForceAccumulator accumulator, RigidBodyIsland island, float dt) {
        for (int i = 0; i < island.getSize(); i++) {
            RigidBody rigidBody = island.getRigidBody(i);

            if (rigidBody.getDynamicsData().getBodyProperties().inverseMass==0)
                continue;

            for (int j = 0; j < island.getSize(); j++) {
                RigidBody rigidBody2 = island.getRigidBody(j);


                if (rigidBody2.getDynamicsData().getBodyProperties().inverseMass==0)
                    continue;

                Vector3f dir = rigidBody2.getPosition().duplicate().sub(rigidBody.getPosition());
                float distSquared = dir.lengthSquared();
                if (distSquared==0)
                    continue;
                dir.normalizeSafely();
                float currentForce = g/(rigidBody.getDynamicsData().getBodyProperties().inverseMass*rigidBody2.getDynamicsData().getBodyProperties().inverseMass*distSquared);

                dir.mul(currentForce);

                cachingForceResult.set(dir, torque);
                accumulator.addForce(cachingForceResult, i);
            }
        }
    }
}
