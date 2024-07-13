package fr.radi3nt.physics.sleeping.provider;

import fr.radi3nt.physics.sleeping.PositionSleepingData;
import fr.radi3nt.physics.sleeping.SleepingData;

public class PositionSleepingDataProvider implements SleepingDataProvider {

    private final float threshold;
    private final int stepRequired;

    public PositionSleepingDataProvider(float threshold, int stepRequired) {
        this.threshold = threshold;
        this.stepRequired = stepRequired;
    }


    @Override
    public SleepingData provide() {
        return new PositionSleepingData(threshold, stepRequired);
    }
}
