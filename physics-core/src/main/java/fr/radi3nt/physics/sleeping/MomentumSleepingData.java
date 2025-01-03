package fr.radi3nt.physics.sleeping;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.core.state.DynamicsData;

public class MomentumSleepingData implements SleepingData {

    private final float linearThreshold;
    private final float angularThreshold;
    private final int stepRequiredToSleep;

    public int stepStatic;

    private Vector3f lastForce;
    private Vector3f lastTorque;

    public boolean sleeping;
    public boolean askedWokeUp;

    public MomentumSleepingData(float linearThreshold, float angularThreshold, int stepRequiredToSleep) {
        this.linearThreshold = linearThreshold;
        this.angularThreshold = angularThreshold;
        this.stepRequiredToSleep = stepRequiredToSleep;
    }

    @Override
    public void ode(Vector3f force, Vector3f torque) {
        if (true)
            return;

        if (lastForce==null || lastForce.duplicate().sub(force).lengthSquared()>= linearThreshold * linearThreshold) {
            sleeping = false;
            lastForce = null;
            lastTorque = null;
        } else if (lastTorque==null || lastTorque.duplicate().sub(torque).lengthSquared()>= linearThreshold * linearThreshold) {
            sleeping = false;
            lastForce = null;
            lastTorque = null;
        }

        if (lastForce==null)
            lastForce = force.duplicate();
        if (lastTorque==null)
            lastTorque = torque.duplicate();
    }

    @Override
    public void step(DynamicsData data) {
        if (askedWokeUp) {
            actuallyWake();
            askedWokeUp = false;
        }

        boolean staticObject = shouldKeepSleeping(data);
        if (staticObject)
            stepStatic++;
        else {
            stepStatic = 0;
            sleeping = false;
        }

        if (shouldSleep()) {
            sleep(data);
        }
    }

    @Override
    public void wakeUpIfNeeded(DynamicsData data) {
        if (!isSleeping())
            return;
        if (shouldKeepSleeping(data))
            return;
        wakeUp();
    }

    private void actuallyWake() {
        stepStatic=0;
        lastForce = null;
        lastTorque = null;
        sleeping = false;
    }

    private void sleep(DynamicsData data) {
        data.zeroAngularMomentum();
        data.zeroLinearMomentum();
        sleeping = true;
    }

    private boolean shouldSleep() {
        return stepStatic>stepRequiredToSleep;
    }

    @Override
    public void wakeUp() {
        askedWokeUp = true;
    }

    @Override
    public boolean isSleeping() {
        return sleeping;
    }

    @Override
    public boolean isAwoken() {
        return askedWokeUp || !sleeping;
    }

    @Override
    public boolean shouldWakeUp(DynamicsData a) {
        return !shouldKeepSleeping(a);
    }

    private boolean shouldKeepSleeping(DynamicsData a) {
        return a.getLinearVelocity().lengthSquared() <= linearThreshold * linearThreshold && a.getAngularVelocity().lengthSquared() <= angularThreshold * angularThreshold;
    }
}
