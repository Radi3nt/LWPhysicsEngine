package fr.radi3nt.physics.constraints.constraint.index;

import fr.radi3nt.physics.core.state.DynamicsData;
import fr.radi3nt.physics.sleeping.SleepingData;

public class IdentifiedDynamicsData {

    public final DynamicsData data;
    public final SleepingData sleepingData;
    private final int id;

    public IdentifiedDynamicsData(DynamicsData data, SleepingData sleepingData, int id) {
        this.data = data;
        this.sleepingData = sleepingData;
        this.id = id;
    }

    public boolean isStatic() {
        return data.getBodyProperties().inverseMass==0 || sleepingData.isSleeping();
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof IdentifiedDynamicsData)) return false;

        IdentifiedDynamicsData that = (IdentifiedDynamicsData) o;

        return id == that.id;
    }

    @Override
    public int hashCode() {
        return id;
    }
}
