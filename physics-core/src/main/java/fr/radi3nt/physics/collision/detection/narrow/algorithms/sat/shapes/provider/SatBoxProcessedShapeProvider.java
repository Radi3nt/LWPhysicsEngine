package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.provider;

import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.box.OffsetSatBoxProcessedShape;
import fr.radi3nt.physics.collision.detection.narrow.processed.ProcessedShapeProvider;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.SatProcessedShape;
import fr.radi3nt.physics.collision.shape.shapes.BoxShape;
import fr.radi3nt.physics.collision.shape.shapes.CollisionShape;
import fr.radi3nt.physics.core.TransformedObject;

public class SatBoxProcessedShapeProvider implements ProcessedShapeProvider<SatProcessedShape> {

    public static final SatBoxProcessedShapeProvider INSTANCE = new SatBoxProcessedShapeProvider();

    private SatBoxProcessedShapeProvider() {
    }

    @Override
    public boolean isSupported(CollisionShape shape) {
        return shape instanceof BoxShape;
    }

    @Override
    public SatProcessedShape getShape(CollisionShape shape, TransformedObject transformedObject) {
        BoxShape boxShape = (BoxShape) shape;
        return new OffsetSatBoxProcessedShape(transformedObject, boxShape.getSize(), boxShape.getOffset(), boxShape.getRotation()).compute();
    }
}
