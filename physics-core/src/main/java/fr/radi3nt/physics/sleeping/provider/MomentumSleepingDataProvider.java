package fr.radi3nt.physics.sleeping.provider;

import fr.radi3nt.physics.sleeping.MomentumSleepingData;
import fr.radi3nt.physics.sleeping.SleepingData;

public class MomentumSleepingDataProvider implements SleepingDataProvider {

    private final float linearThreshold;
    private final float angularThreshold;
    private final int stepRequired;

    public MomentumSleepingDataProvider(float threshold, float angularThreshold, int stepRequired) {
        this.linearThreshold = threshold;
        this.angularThreshold = angularThreshold;
        this.stepRequired = stepRequired;
    }


    @Override
    public SleepingData provide() {
        return new MomentumSleepingData(linearThreshold, angularThreshold, stepRequired);
    }
}
