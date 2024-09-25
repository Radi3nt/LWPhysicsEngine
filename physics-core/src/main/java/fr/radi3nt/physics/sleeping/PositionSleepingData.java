package fr.radi3nt.physics.sleeping;

import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.core.state.DynamicsData;

public class PositionSleepingData implements SleepingData {

    private final float threshold;
    private final int stepRequiredToSleep;

    private Vector3f lastPos;
    private Quaternion lastRot;

    private Vector3f lastForce;
    private Vector3f lastTorque;

    public int stepStatic;
    private boolean sleeping;
    private boolean askAwake;

    public PositionSleepingData(float threshold, int stepRequiredToSleep) {
        this.threshold = threshold;
        this.stepRequiredToSleep = stepRequiredToSleep;
    }

    @Override
    public void ode(Vector3f force, Vector3f torque) {
        if (lastForce==null || lastForce.duplicate().sub(force).lengthSquared()>=threshold*threshold) {
            sleeping = false;
            lastForce = null;
            lastTorque = null;
        } else if (lastTorque==null || lastTorque.duplicate().sub(torque).lengthSquared()>=threshold*threshold) {
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
        if (askAwake) {
            actuallyWakeUp();
            askAwake = false;
        }

        boolean staticObject = lastPos!=null && lastRot!=null && lastPos.duplicate().sub(data.getPosition()).lengthSquared()<threshold*threshold;
        if (staticObject) {
            Quaternion quat = lastRot.duplicate();
            quat.multiply(data.getRotation().duplicate().getConjugate());
            if (quat.getW()<1-threshold) {
                staticObject = false;
            }
        }
        if (staticObject)
            stepStatic++;
        else {
            stepStatic = 0;
            lastPos = null;
            lastRot = null;
            sleeping = false;
        }

        if (lastPos==null)
            lastPos = data.getPosition().duplicate();
        if (lastRot==null)
            lastRot = data.getRotation().duplicate();

        if (shouldSleep()) {
            sleep();
        }
    }

    @Override
    public void wakeUpIfNeeded(DynamicsData data) {
        if (!isSleeping())
            return;
        if (shouldWakeUp(data) || (data.getLinearMomentum().lengthSquared()!=0 || data.getAngularMomentum().lengthSquared()!=0))
            wakeUp();
    }

    private void sleep() {
        sleeping = true;
    }

    private boolean shouldSleep() {
        return stepStatic>stepRequiredToSleep;
    }

    @Override
    public void wakeUp() {
        askAwake = true;
    }

    private void actuallyWakeUp() {
        stepStatic=0;
        lastPos = null;
        lastRot = null;
        lastForce = null;
        lastTorque = null;
        sleeping = false;
    }

    @Override
    public boolean isSleeping() {
        return sleeping;
    }

    @Override
    public boolean isAwoken() {
        return !sleeping;
    }

    @Override
    public boolean shouldWakeUp(DynamicsData data) {
        boolean staticObject = lastPos!=null && lastRot!=null && lastPos.duplicate().sub(data.getPosition()).lengthSquared()<threshold*threshold;
        return !staticObject;
    }
}
