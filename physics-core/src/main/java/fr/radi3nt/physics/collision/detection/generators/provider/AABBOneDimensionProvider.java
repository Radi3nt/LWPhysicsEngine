package fr.radi3nt.physics.collision.detection.generators.provider;

import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.AABB;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.mapping.AxisMapping;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.CachingAABB;
import fr.radi3nt.physics.collision.detection.broad.pre.PreCollisionShape;
import fr.radi3nt.physics.core.TransformedObject;

public class AABBOneDimensionProvider implements OneDimensionProvider {

    @Override
    public AxisMapping map(TransformedObject body, PreCollisionShape preCollisionShape) {
        AABB aabb = preCollisionShape.getBoundingBox(body);
        return aabb.getxMapping();
    }
}
