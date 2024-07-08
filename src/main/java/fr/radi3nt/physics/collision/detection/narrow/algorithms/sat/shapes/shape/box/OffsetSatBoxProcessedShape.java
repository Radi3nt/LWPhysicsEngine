package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.box;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.core.TransformedObject;

public class OffsetSatBoxProcessedShape extends SatBoxProcessedShape {

    private final Vector3f offset;

    public OffsetSatBoxProcessedShape(TransformedObject transformedObject, Vector3f size, Vector3f offset) {
        super(transformedObject, size);
        this.offset = offset;
    }

    @Override
    public OffsetSatBoxProcessedShape compute() {
        return (OffsetSatBoxProcessedShape) super.compute();
    }

    @Override
    protected void computeVertices() {
        super.computeVertices();
        for (Vector3f vertex : vertices) {
            vertex.add(offset);
        }
    }
}
