package fr.radi3nt.physics.collision.detection.broad.aabb;

import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.AABB;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.mapping.AxisMapping;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.SetAABB;
import fr.radi3nt.physics.collision.detection.broad.BroadPhaseDetectionStrategy;
import fr.radi3nt.physics.collision.detection.broad.aabb.overlap.OverlapTest;
import fr.radi3nt.physics.collision.detection.broad.aabb.overlap.OverlapXTest;
import fr.radi3nt.physics.collision.detection.broad.aabb.overlap.OverlapYTest;
import fr.radi3nt.physics.collision.detection.broad.aabb.overlap.OverlapZTest;
import fr.radi3nt.physics.collision.detection.broad.pre.PreCollisionPair;
import fr.radi3nt.physics.collision.detection.broad.pre.PreCollisionShape;
import fr.radi3nt.physics.collision.detection.narrow.processed.ProcessedShapeProvider;
import fr.radi3nt.physics.core.TransformedObject;

public class AABBBroadPhaseDetectionStrategy implements BroadPhaseDetectionStrategy {

    private final OverlapTest[] overlapTests;

    private final SetAABB aabbA = SetAABB.zero();
    private final SetAABB aabbB = SetAABB.zero();

    public AABBBroadPhaseDetectionStrategy(OverlapTest[] overlapTests) {
        this.overlapTests = overlapTests;
    }

    public static AABBBroadPhaseDetectionStrategy standard() {
        return new AABBBroadPhaseDetectionStrategy(new OverlapTest[]{new OverlapXTest(), new OverlapYTest(), new OverlapZTest()});
    }

    @Override
    public boolean canSkip(PreCollisionPair pair) {
        return canSkip(pair.getShapeA(), pair.getShapeB(), pair.getBodyA(), pair.getBodyB());
    }

    private boolean canSkip(PreCollisionShape shapeA, PreCollisionShape shapeB, TransformedObject bodyA, TransformedObject bodyB) {
        aabbA.copy(shapeA.getBoundingBox(bodyA));
        aabbB.copy(shapeB.getBoundingBox(bodyB));

        boolean canSkip = false;
        for (OverlapTest overlapTest : overlapTests) {
            if (overlapTest.noOverlap(this.aabbA, this.aabbB)) {
                canSkip = true;
                break;
            }
        }

        return canSkip;
    }

    private boolean noOverlapX(AABB aabbA, AABB aabbB) {
        return no(overlap(aabbA.getxMapping(), aabbB.getxMapping()));
    }

    private boolean noOverlapY(AABB aabbA, AABB aabbB) {
        return no(overlap(aabbA.getyMapping(), aabbB.getyMapping()));
    }

    private boolean noOverlapZ(AABB aabbA, AABB aabbB) {
        return no(overlap(aabbA.getzMapping(), aabbB.getzMapping()));
    }

    private boolean overlap(AxisMapping mappingA, AxisMapping mappingB) {
        return mappingA.intersect(mappingB);
    }

    private boolean no(boolean overlap) {
        return !overlap;
    }
}
