package fr.radi3nt.physics.collision.detection.gen.provider;

import fr.radi3nt.maths.aabb.AxisMapping;
import fr.radi3nt.physics.collision.CollisionShapeGroup;
import fr.radi3nt.physics.collision.detection.broad.aabb.CachingAABB;
import fr.radi3nt.physics.core.TransformedObject;
import fr.radi3nt.physics.core.state.RigidBody;

public class AABBOneDimensionProvider implements OneDimensionProvider {

    @Override
    public AxisMapping map(TransformedObject body, CollisionShapeGroup shapeGroup) {
        CachingAABB aabb = shapeGroup.getPreCollisionShape().getAABB();
        aabb.prepare(body.getPosition(), body.getRotation());
        return aabb.getxMapping();
    }
}
