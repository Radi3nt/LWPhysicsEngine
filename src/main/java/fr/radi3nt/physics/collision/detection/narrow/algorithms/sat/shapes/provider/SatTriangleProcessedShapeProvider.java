package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.provider;

import fr.radi3nt.physics.collision.detection.narrow.processed.ProcessedShapeProvider;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.SatProcessedShape;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.triangles.SatCreatingTriangleProcessedShape;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.triangles.SatOneWayEdgelessCreatingTriangleProcessedShape;
import fr.radi3nt.physics.collision.shape.shapes.CollisionShape;
import fr.radi3nt.physics.collision.shape.shapes.TriangleShape;
import fr.radi3nt.physics.core.TransformedObject;

public class SatTriangleProcessedShapeProvider implements ProcessedShapeProvider<SatProcessedShape> {

    public static final SatTriangleProcessedShapeProvider INSTANCE = new SatTriangleProcessedShapeProvider();

    private SatTriangleProcessedShapeProvider() {
    }

    @Override
    public boolean isSupported(CollisionShape shape) {
        return shape instanceof TriangleShape;
    }

    @Override
    public SatProcessedShape getShape(CollisionShape shape, TransformedObject transformedObject) {
        TriangleShape triangleShape = (TriangleShape) shape;
        if (triangleShape.isEdgeless() && triangleShape.isOneWay())
            return new SatOneWayEdgelessCreatingTriangleProcessedShape(transformedObject, triangleShape.getVertex1(), triangleShape.getVertex2(), triangleShape.getVertex3());
        return new SatCreatingTriangleProcessedShape(transformedObject, triangleShape.getVertex1(), triangleShape.getVertex2(), triangleShape.getVertex3());
    }
}
