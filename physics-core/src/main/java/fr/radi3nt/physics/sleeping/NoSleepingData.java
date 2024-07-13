package fr.radi3nt.physics.sleeping;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.core.state.DynamicsData;

public class NoSleepingData implements SleepingData {
    @Override
    public void ode(Vector3f force, Vector3f torque) {

    }

    @Override
    public void step(DynamicsData data) {

    }

    @Override
    public void wakeUp() {

    }

    @Override
    public boolean isSleeping() {
        return false;
    }

    @Override
    public boolean isAwoken() {
        return true;
    }

    @Override
    public boolean shouldWakeUp(DynamicsData a) {
        return true;
    }
}
