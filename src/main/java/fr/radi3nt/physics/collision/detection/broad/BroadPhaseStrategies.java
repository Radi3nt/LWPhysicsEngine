package fr.radi3nt.physics.collision.detection.broad;

import fr.radi3nt.physics.collision.CollisionShapeGroup;
import fr.radi3nt.physics.collision.shape.CollisionShape;
import fr.radi3nt.physics.collision.shape.pre.PreCollisionPair;
import fr.radi3nt.physics.collision.shape.pre.PreCollisionShape;
import fr.radi3nt.physics.core.TransformedObject;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.function.BiPredicate;
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
