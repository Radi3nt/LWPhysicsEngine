package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.provider;

import fr.radi3nt.physics.collision.detection.narrow.processed.ProcessedShapeProvider;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.SatProcessedShape;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.SatSquareProcessedShape;
import fr.radi3nt.physics.collision.shape.shapes.CollisionShape;
import fr.radi3nt.physics.collision.shape.shapes.SquareShape;
import fr.radi3nt.physics.core.TransformedObject;

public class SatSquareProcessedShapeProvider implements ProcessedShapeProvider<SatProcessedShape> {

    public static final SatSquareProcessedShapeProvider INSTANCE = new SatSquareProcessedShapeProvider();

    private SatSquareProcessedShapeProvider() {
    }

    @Override
    public boolean isSupported(CollisionShape shape) {
        return shape instanceof SquareShape;
    }

    @Override
    public SatProcessedShape getShape(CollisionShape shape, TransformedObject transformedObject) {
        return new SatSquareProcessedShape(((SquareShape) shape).getSize()).compute();
    }
}
