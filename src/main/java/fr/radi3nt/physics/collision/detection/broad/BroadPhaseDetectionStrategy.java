package fr.radi3nt.physics.collision.detection.broad;

import fr.radi3nt.physics.collision.shape.pre.PreCollisionPair;

public interface BroadPhaseDetectionStrategy {

    boolean canSkip(PreCollisionPair pair);

}
