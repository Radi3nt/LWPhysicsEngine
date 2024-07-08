package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.projection;

import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.mapping.ProjectionSettable;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.pool.ObjectPool;

public interface SatProjectionProvider {

    void project(ProjectionSettable p, Vector3f axis);
    void free(ObjectPool<Vector3f> pool);

}
