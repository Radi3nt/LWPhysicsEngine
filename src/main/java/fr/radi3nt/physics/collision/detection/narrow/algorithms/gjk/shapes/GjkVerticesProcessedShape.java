package fr.radi3nt.physics.collision.detection.narrow.algorithms.gjk.shapes;

import fr.radi3nt.maths.components.vectors.Vector3f;

import java.util.ArrayList;
import java.util.List;

public class GjkVerticesProcessedShape implements GjkProcessedShape {

    private final Vector3f[] transformedVertices;

    public GjkVerticesProcessedShape(Vector3f[] transformedVertices) {
        this.transformedVertices = transformedVertices;
    }

    @Override
    public List<FurthestPoint> furthestPointAlongAxis(Vector3f axis) {
        float maxDistance = Float.NEGATIVE_INFINITY;
        List<FurthestPoint> furthestPoints = new ArrayList<>();
        for (int i = 0, transformedVerticesLength = transformedVertices.length; i < transformedVerticesLength; i++) {
            Vector3f transformedVertex = transformedVertices[i];
            float currentDistance = axis.dot(transformedVertex);
            if (maxDistance <= currentDistance) {
                if (maxDistance!=currentDistance) {
                    maxDistance = currentDistance;
                    furthestPoints.clear();
                }
                furthestPoints.add(new FurthestPoint(transformedVertex, i));
            }
        }
        return furthestPoints;
    }

    @Override
    public float transformDistance(float dist) {
        return dist;
    }
}
