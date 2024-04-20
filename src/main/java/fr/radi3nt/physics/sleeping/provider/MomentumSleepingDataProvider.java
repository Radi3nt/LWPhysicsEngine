package fr.radi3nt.physics.sleeping.provider;

import fr.radi3nt.physics.sleeping.MomentumSleepingData;
import fr.radi3nt.physics.sleeping.SleepingData;

public class MomentumSleepingDataProvider implements SleepingDataProvider {

    private final float threshold;
    private final int stepRequired;

    public MomentumSleepingDataProvider(float threshold, int stepRequired) {
        this.threshold = threshold;
        this.stepRequired = stepRequired;
    }


    @Override
    public SleepingData provide() {
        return new MomentumSleepingData(threshold, stepRequired);
    }
}
