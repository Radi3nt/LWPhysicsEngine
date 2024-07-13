package fr.radi3nt.physics.collision.detection.narrow.processed;

import fr.radi3nt.physics.collision.shape.shapes.CollisionShape;
import fr.radi3nt.physics.core.TransformedObject;

public interface ProcessedShapeProvider<T extends ProcessedShape> {

    boolean isSupported(CollisionShape shape);
    T getShape(CollisionShape shape, TransformedObject transformedObject);

}
