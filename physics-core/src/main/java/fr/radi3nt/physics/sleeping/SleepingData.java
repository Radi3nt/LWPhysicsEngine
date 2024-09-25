package fr.radi3nt.physics.sleeping;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.core.state.DynamicsData;

public interface SleepingData {

    void ode(Vector3f force, Vector3f torque);
    void step(DynamicsData data);
    void wakeUpIfNeeded(DynamicsData data);
    void wakeUp();
    boolean isSleeping();
    boolean isAwoken();

    boolean shouldWakeUp(DynamicsData a);

    default void wakeUpIfNeeded() {
        if (isSleeping())
            wakeUp();
    }
}
