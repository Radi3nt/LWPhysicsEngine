package fr.radi3nt.physics.collision.shape.sat.shape.box;

import fr.radi3nt.maths.components.vectors.Vector3f;

public class OffsetSatBoxShapeObject extends SatBoxShapeObject {

    private final Vector3f offset;

    public OffsetSatBoxShapeObject(Vector3f size, Vector3f offset) {
        super(size);
        this.offset = offset;
    }

    @Override
    public OffsetSatBoxShapeObject compute() {
        return (OffsetSatBoxShapeObject) super.compute();
    }

    @Override
    protected void computeVertices() {
        super.computeVertices();
        for (Vector3f vertex : vertices) {
            vertex.add(offset);
        }
    }
}
