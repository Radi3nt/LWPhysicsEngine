package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.triangles;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.computer.NormalSatCollisionDetector;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.Edge;

public class SatOneWayEdgelessTriangleProcessedShape extends SatTriangleProcessedShape {

    public SatOneWayEdgelessTriangleProcessedShape(Vector3f first, Vector3f second, Vector3f third) {
        super(first, second, third);
    }

    @Override
    public Edge[] getClipEdges() {
        return new Edge[0];
    }

    @Override
    public int transformNormal(Vector3f mta, int normal, boolean isA, NormalSatCollisionDetector.SatContactType contactType) {
        if (mta.dot(this.normal)*normal>0) {
            return -normal;
        }
        return normal;
    }
}
