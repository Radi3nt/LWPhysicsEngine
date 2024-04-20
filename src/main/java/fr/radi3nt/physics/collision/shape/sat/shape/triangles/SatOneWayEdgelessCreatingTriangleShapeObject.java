package fr.radi3nt.physics.collision.shape.sat.shape.triangles;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.shape.sat.clip.Edge;

public class SatOneWayEdgelessCreatingTriangleShapeObject extends SatCreatingTriangleShapeObject {

    public SatOneWayEdgelessCreatingTriangleShapeObject(Vector3f first, Vector3f second, Vector3f third, Vector3f cache) {
        super(first, second, third, cache);
    }

    @Override
    public Edge[] getClipEdges() {
        return new Edge[0];
    }

    @Override
    public int transformNormal(Vector3f mta, int normal) {
        if (mta.dot(this.normal)*normal<0) {
            return normal*-1;
        }
        return normal;
    }
}
