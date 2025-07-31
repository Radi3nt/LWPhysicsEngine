package fr.radi3nt.physics.dynamics.force.caster;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.force.accumulator.MotionAccumulator;
import fr.radi3nt.physics.dynamics.force.accumulator.MotionResult;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

import java.util.function.Supplier;

public class SetAttractionForceCaster implements ForceCaster {

    private final MotionResult cachingForceResult = new MotionResult();
    private final float g;
    private final Supplier<Vector3f> position;
    private final Vector3f torque;

    public SetAttractionForceCaster(float g, Supplier<Vector3f> position, Vector3f torque) {
        this.g = g;
        this.position = position;
        this.torque = torque;
    }

    @Override
    public void cast(MotionAccumulator accumulator, RigidBodyIsland island, float dt, int index) {
        RigidBody rigidBody = island.getRigidBody(index);

        if (rigidBody.getDynamicsData().getBodyProperties().inverseMass==0)
            return;


        Vector3f dir = position.get().duplicate().sub(rigidBody.getPosition());
        float distSquared = dir.lengthSquared();
        if (distSquared==0)
            return;
        dir.normalizeSafely();
        float currentForce = g/(rigidBody.getDynamicsData().getBodyProperties().inverseMass*distSquared);

        dir.mul(currentForce);

        cachingForceResult.set(dir, torque);
        accumulator.addMotion(cachingForceResult, index);
    }
}
