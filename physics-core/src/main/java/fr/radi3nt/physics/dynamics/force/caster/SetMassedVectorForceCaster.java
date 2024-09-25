package fr.radi3nt.physics.dynamics.force.caster;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.force.accumulator.MotionAccumulator;
import fr.radi3nt.physics.dynamics.force.accumulator.MotionResult;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

public class SetMassedVectorForceCaster implements ForceCaster {

    private final MotionResult cachingForceResult = new MotionResult();
    private final Vector3f force;
    private final Vector3f torque;

    public SetMassedVectorForceCaster(Vector3f force, Vector3f torque) {
        this.force = force;
        this.torque = torque;
    }

    @Override
    public void cast(MotionAccumulator accumulator, RigidBodyIsland island, float dt, int index) {
        RigidBody rigidBody = island.getRigidBody(index);
        Vector3f currentForce = force.duplicate();
        float inverseMass = rigidBody.getDynamicsData().getBodyProperties().inverseMass;
        if (inverseMass>0)
            currentForce.mul(1/inverseMass);
        cachingForceResult.set(currentForce, torque);
        accumulator.addMotion(cachingForceResult, index);
    }
}
