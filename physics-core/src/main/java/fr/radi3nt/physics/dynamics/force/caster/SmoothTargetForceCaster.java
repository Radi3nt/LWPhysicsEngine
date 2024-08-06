package fr.radi3nt.physics.dynamics.force.caster;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.maths.dynamics.DynamicsConstants;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.force.accumulator.MotionAccumulator;
import fr.radi3nt.physics.dynamics.force.accumulator.MotionResult;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

public class SmoothTargetForceCaster implements ForceCaster {

    private final DynamicsConstants constants;
    private final Vector3f inputCurrent;
    private final Vector3f inputPrevious = new SimpleVector3f();
    private float lastDt = 1;

    private final MotionResult motionResult = new MotionResult();

    public SmoothTargetForceCaster(DynamicsConstants constants, Vector3f inputCurrent) {
        this.constants = constants;
        this.inputCurrent = inputCurrent;
        this.inputPrevious.copy(inputCurrent);
    }

    public SmoothTargetForceCaster(DynamicsConstants constants) {
        this(constants, new SimpleVector3f());
    }

    @Override
    public void cast(MotionAccumulator motionAccumulator, RigidBodyIsland island, float dt, int index) {
        if (dt==0)
            return;

        RigidBody rigidBody = island.getRigidBody(index);
        Vector3f responseDerivative = rigidBody.getDynamicsData().getLinearMomentum();
        Vector3f response = rigidBody.getPosition();

        Vector3f inputDerivative = (inputCurrent.duplicate().sub(inputPrevious).div(lastDt));

        float k1Stable, k2Stable;
        if (systemIsAtHighSpeed(constants, dt)) {
            k1Stable = constants.getK1();
            k2Stable = Math.max(constants.getK2(), Math.max(dt*dt/2 + dt*k1Stable/2, dt*k1Stable));
        } else {
            k1Stable = constants.getK1();
            k2Stable = constants.getK2();
        }

        Vector3f multipliedByTTerm = inputDerivative.duplicate().mul(constants.getK3()).add(inputCurrent).sub(response).mul(dt);
        Vector3f applyingForce = (multipliedByTTerm.duplicate().add(responseDerivative.duplicate().mul(k2Stable))).div(k2Stable+dt*k1Stable);
        applyingForce.sub(responseDerivative);
        applyingForce.div(dt);
        applyingForce.mul(rigidBody.getDynamicsData().getBodyProperties().inverseMass);

        motionResult.set(applyingForce, new SimpleVector3f());
        motionAccumulator.addMotion(motionResult, index);
    }

    private boolean systemIsAtHighSpeed(DynamicsConstants constants, float step) {
        return constants.getW()*step >= constants.getZ();
    }

    @Override
    public void step(RigidBodyIsland island, float dt) {
        inputPrevious.copy(inputCurrent);
        lastDt = dt;
    }
}
