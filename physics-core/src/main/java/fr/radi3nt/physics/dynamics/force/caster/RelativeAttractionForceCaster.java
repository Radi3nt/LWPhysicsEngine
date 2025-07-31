package fr.radi3nt.physics.dynamics.force.caster;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.force.accumulator.MotionAccumulator;
import fr.radi3nt.physics.dynamics.force.accumulator.MotionResult;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

import java.util.function.Supplier;

public class RelativeAttractionForceCaster implements ForceCaster {

    private final MotionResult cachingForceResult = new MotionResult();
    private final float attraction;
    private final Supplier<Vector3f> position;
    private final Vector3f torque;

    public RelativeAttractionForceCaster(float attraction, Supplier<Vector3f> position, Vector3f torque) {
        this.attraction = attraction;
        this.position = position;
        this.torque = torque;
    }

    @Override
    public void cast(MotionAccumulator accumulator, RigidBodyIsland island, float dt, int index) {
        RigidBody rigidBody = island.getRigidBody(index);

        if (rigidBody.getDynamicsData().getBodyProperties().inverseMass==0)
            return;


        Vector3f dir = position.get().duplicate().sub(rigidBody.getPosition());
        dir.normalizeSafely();
        float currentForce = attraction /(rigidBody.getDynamicsData().getBodyProperties().inverseMass);

        dir.mul(currentForce);

        cachingForceResult.set(dir, torque);
        accumulator.addMotion(cachingForceResult, index);
    }
}
