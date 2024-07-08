package fr.radi3nt.physics.collision.detection.narrow.algorithms.gjk.shapes;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.detection.narrow.processed.ProcessedShape;

import java.util.List;

public interface GjkProcessedShape extends ProcessedShape {

    List<FurthestPoint> furthestPointAlongAxis(Vector3f axis);
    float transformDistance(float dist);

    class FurthestPoint {

        public final Vector3f point;
        public final int index;

        public FurthestPoint(Vector3f point, int index) {
            this.point = point;
            this.index = index;
        }
    }

}
