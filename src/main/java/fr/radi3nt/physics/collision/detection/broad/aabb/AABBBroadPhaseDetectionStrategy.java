package fr.radi3nt.physics.collision.detection.broad.aabb;

import fr.radi3nt.maths.aabb.AABB;
import fr.radi3nt.maths.aabb.AxisMapping;
import fr.radi3nt.maths.aabb.SetAABB;
import fr.radi3nt.physics.collision.detection.broad.BroadPhaseDetectionStrategy;
import fr.radi3nt.physics.collision.detection.broad.aabb.overlap.OverlapTest;
import fr.radi3nt.physics.collision.detection.broad.aabb.overlap.OverlapXTest;
import fr.radi3nt.physics.collision.detection.broad.aabb.overlap.OverlapYTest;
import fr.radi3nt.physics.collision.detection.broad.aabb.overlap.OverlapZTest;
import fr.radi3nt.physics.collision.shape.pre.PreCollisionPair;
import fr.radi3nt.physics.collision.shape.pre.PreCollisionShape;
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
        CachingAABB aabbA = shapeA.getAABB();
        CachingAABB aabbB = shapeB.getAABB();

        aabbA.toSetAABB(this.aabbA, bodyA.getPosition(), bodyA.getRotation());
        aabbB.toSetAABB(this.aabbB, bodyB.getPosition(), bodyB.getRotation());

        //aabbA.prepare(bodyA.getPosition(), bodyA.getRotation());
        //aabbB.prepare(bodyB.getPosition(), bodyB.getRotation());
        boolean canSkip = false;
        for (OverlapTest overlapTest : overlapTests) {
            if (overlapTest.noOverlap(this.aabbA, this.aabbB)) {
                canSkip = true;
                break;
            }
        }

        //aabbA.release();
        //aabbB.release();

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
