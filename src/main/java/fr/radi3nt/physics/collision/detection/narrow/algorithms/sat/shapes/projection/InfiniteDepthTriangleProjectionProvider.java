package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.projection;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.mapping.ProjectionSettable;

public class InfiniteDepthTriangleProjectionProvider extends VerticesProjectionProvider {

    private final Vector3f normal;

    public InfiniteDepthTriangleProjectionProvider(Vector3f[] transformedVertices, Vector3f normal) {
        super(transformedVertices);
        this.normal = normal;
    }


    @Override
    public void project(ProjectionSettable pSet, Vector3f axis) {
        super.project(pSet, axis);

        float dot = axis.dot(normal);
        if (dot==0)
            return;
        if (axis.dot(normal)<0) {
            pSet.setMax(Float.MAX_VALUE);
        } else {
            pSet.setMin(-Float.MAX_VALUE);
        }
    }
}
