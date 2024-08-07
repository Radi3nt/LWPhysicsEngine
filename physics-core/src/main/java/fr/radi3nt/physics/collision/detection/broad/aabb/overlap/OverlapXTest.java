package fr.radi3nt.physics.collision.detection.broad.aabb.overlap;

import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.AABB;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.mapping.AxisMapping;

public class OverlapXTest implements OverlapTest {

    private final AxisMapping firstMapping = new AxisMapping(0, 0);
    private final AxisMapping secondMapping = new AxisMapping(0, 0);

    @Override
    public boolean noOverlap(AABB aabbA, AABB aabbB) {
        return !(overlap(aabbA.getxMapping(firstMapping), aabbB.getxMapping(secondMapping)));
    }
}
