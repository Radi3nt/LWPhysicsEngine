package fr.radi3nt.physics.collision.detection.broad.aabb.shapes;

import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.AABB;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.OBBDynamicAABB;
import fr.radi3nt.physics.collision.detection.narrow.processed.ProcessedShapeProvider;
import fr.radi3nt.physics.collision.shape.shapes.CapsuleShape;
import fr.radi3nt.physics.collision.shape.shapes.CollisionShape;
import fr.radi3nt.physics.core.TransformedObject;

public class CapsuleAABBProcessedShapeProvider implements ProcessedShapeProvider<AABB> {
    @Override
    public boolean isSupported(CollisionShape shape) {
        return shape instanceof CapsuleShape;
    }

    @Override
    public AABB getShape(CollisionShape shape, TransformedObject transformedObject) {
        CapsuleShape capsuleShape = (CapsuleShape) shape;
        return new OBBDynamicAABB(transformedObject, new SimpleVector3f(capsuleShape.getRadius(), capsuleShape.getHeight()+ capsuleShape.getRadius()*2, capsuleShape.getRadius()), capsuleShape.getOffset(), capsuleShape.getRotation());
    }
}
