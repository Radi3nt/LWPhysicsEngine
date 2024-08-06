package fr.radi3nt.physics.dynamics.force.caster;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.force.accumulator.MotionAccumulator;
import fr.radi3nt.physics.dynamics.force.accumulator.MotionResult;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

public class TorqueDampForceCaster implements ForceCaster {

    private final MotionResult cachingForceResult = new MotionResult();
    private final float damping;

    public TorqueDampForceCaster(float damping) {
        this.damping = damping;
    }

    @Override
    public void cast(MotionAccumulator accumulator, RigidBodyIsland island, float dt, int index) {
        RigidBody rigidBody = island.getRigidBody(index);

        Vector3f dragTorque = getDragTorque(rigidBody);
        cachingForceResult.set(new SimpleVector3f(), dragTorque);
        accumulator.addMotion(cachingForceResult, index);
    }

    private Vector3f getDragTorque(RigidBody rigidBody) {
        Vector3f relativeVel = rigidBody.getDynamicsData().getAngularMomentum();
        Vector3f dragTorque = relativeVel.duplicate().mul(relativeVel);
        dragTorque.mul(Math.signum(relativeVel.getX()), Math.signum(relativeVel.getY()), Math.signum(relativeVel.getZ()));
        dragTorque.mul(-1/2f*damping);
        return dragTorque;
    }
}
