package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.triangles;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.computer.detector.NormalSatCollisionDetector;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.Edge;
import fr.radi3nt.physics.core.TransformedObject;

public class SatOneWayEdgelessCreatingTriangleProcessedShape extends SatCreatingTriangleProcessedShape {

    public SatOneWayEdgelessCreatingTriangleProcessedShape(TransformedObject object, Vector3f first, Vector3f second, Vector3f third) {
        super(object, first, second, third);
    }

    @Override
    public Edge[] getClipEdges() {
        return new Edge[0];
    }

    @Override
    public int transformNormal(Vector3f mta, int normal, boolean isA, NormalSatCollisionDetector.SatContactType contactType) {
        if (!contactType.isFace())
            return normal;
        if (mta.dot(this.normal)*normal*(isA? 1 : -1)<0) {
            return -normal;
        }
        return normal;
    }

    @Override
    public boolean canUseEdgesAsMinimum() {
        return false;
    }
}
