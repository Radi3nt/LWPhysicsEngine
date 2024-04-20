package fr.radi3nt.physics.collision.detection.broad.aabb.overlap;

import fr.radi3nt.maths.aabb.AABB;
import fr.radi3nt.maths.aabb.AxisMapping;

public interface OverlapTest {

    boolean noOverlap(AABB aabbA, AABB aabbB);

    default boolean overlap(AxisMapping mappingA, AxisMapping mappingB) {
        return mappingA.intersect(mappingB);
    }

}
