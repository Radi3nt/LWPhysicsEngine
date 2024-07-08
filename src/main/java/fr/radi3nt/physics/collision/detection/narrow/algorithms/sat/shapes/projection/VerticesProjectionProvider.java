package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.projection;

import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.mapping.ProjectionSettable;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.pool.ObjectPool;

public class VerticesProjectionProvider implements SatProjectionProvider {

    private final Vector3f[] transformedVertices;

    public VerticesProjectionProvider(Vector3f[] transformedVertices) {
        this.transformedVertices = transformedVertices;
    }

    @Override
    public void project(ProjectionSettable pSet, Vector3f axis) {
        float min = axis.dot(transformedVertices[0]);
        float max = min;
        for (int i = 1; i < transformedVertices.length; i++) {
            float p = axis.dot(transformedVertices[i]);
            min = Math.min(p, min);
            max = Math.max(p, max);
        }

        pSet.set(min, max);
    }

    @Override
    public void free(ObjectPool<Vector3f> pool) {

    }
}
