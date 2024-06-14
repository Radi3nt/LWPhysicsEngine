package fr.radi3nt.physics.sleeping.provider;

import fr.radi3nt.physics.sleeping.NoSleepingData;
import fr.radi3nt.physics.sleeping.SleepingData;

public class NoSleepingDataProvider implements SleepingDataProvider {


    @Override
    public SleepingData provide() {
        return new NoSleepingData();
    }
}
