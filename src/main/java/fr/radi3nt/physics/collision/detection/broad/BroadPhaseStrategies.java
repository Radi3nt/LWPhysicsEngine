package fr.radi3nt.physics.collision.detection.broad;

import fr.radi3nt.physics.collision.shape.pre.PreCollisionPair;

import java.util.function.Predicate;

public class BroadPhaseStrategies implements Predicate<PreCollisionPair> {

    private final BroadPhaseDetectionStrategy[] strategies;

    public BroadPhaseStrategies(BroadPhaseDetectionStrategy... strategies) {
        this.strategies = strategies;
    }

    @Override
    public boolean test(PreCollisionPair pair) {
        for (BroadPhaseDetectionStrategy strategy : strategies) {
            if (strategy.canSkip(pair))
                return true;
        }
        return false;
    }
}
