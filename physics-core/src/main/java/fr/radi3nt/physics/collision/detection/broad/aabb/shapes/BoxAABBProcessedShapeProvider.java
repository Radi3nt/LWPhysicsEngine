package fr.radi3nt.physics.collision.detection.broad.aabb.shapes;

import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.AABB;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.OBBDynamicAABB;
import fr.radi3nt.physics.collision.detection.narrow.processed.ProcessedShapeProvider;
import fr.radi3nt.physics.collision.shape.shapes.BoxShape;
import fr.radi3nt.physics.collision.shape.shapes.CollisionShape;
import fr.radi3nt.physics.core.TransformedObject;

public class BoxAABBProcessedShapeProvider implements ProcessedShapeProvider<AABB> {

    public static BoxAABBProcessedShapeProvider INSTANCE = new BoxAABBProcessedShapeProvider();

    private BoxAABBProcessedShapeProvider() {
    }

    @Override
    public boolean isSupported(CollisionShape shape) {
        return shape instanceof BoxShape;
    }

    @Override
    public AABB getShape(CollisionShape shape, TransformedObject transformedObject) {
        BoxShape boxShape = (BoxShape) shape;
        return new OBBDynamicAABB(transformedObject, boxShape.getSize(), boxShape.getOffset(), boxShape.getRotation());
    }
}
