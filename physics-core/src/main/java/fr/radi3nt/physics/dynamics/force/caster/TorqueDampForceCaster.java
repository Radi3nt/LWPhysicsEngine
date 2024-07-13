package fr.radi3nt.physics.dynamics.force.caster;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.force.accumulator.ForceAccumulator;
import fr.radi3nt.physics.dynamics.force.accumulator.ForceResult;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

public class TorqueDampForceCaster implements ForceCaster {

    private final ForceResult cachingForceResult = new ForceResult();
    private final float damping;

    public TorqueDampForceCaster(float damping) {
        this.damping = damping;
    }

    @Override
    public void cast(ForceAccumulator accumulator, RigidBodyIsland island, float dt) {
        for (int i = 0; i < island.getSize(); i++) {
            RigidBody rigidBody = island.getRigidBody(i);

            Vector3f dragTorque = getDragTorque(rigidBody);
            cachingForceResult.set(new SimpleVector3f(), dragTorque);
            accumulator.addForce(cachingForceResult, i);
        }
    }

    private Vector3f getDragTorque(RigidBody rigidBody) {
        Vector3f relativeVel = rigidBody.getDynamicsData().getAngularMomentum();
        Vector3f dragTorque = relativeVel.duplicate().mul(relativeVel);
        dragTorque.mul(Math.signum(relativeVel.getX()), Math.signum(relativeVel.getY()), Math.signum(relativeVel.getZ()));
        dragTorque.mul(-1/2f*damping);
        return dragTorque;
    }
}
