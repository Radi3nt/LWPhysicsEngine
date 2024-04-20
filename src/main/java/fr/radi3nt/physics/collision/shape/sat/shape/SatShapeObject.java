package fr.radi3nt.physics.collision.shape.sat.shape;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.shape.sat.clip.ClipPlanes;
import fr.radi3nt.physics.collision.shape.sat.clip.Edge;

public interface SatShapeObject {

    Vector3f[] getAxis();
    Vector3f[] getEdges();
    ClipPlanes getClipPlanes();
    Edge[] getClipEdges();

    Vector3f[] getVertices();
    float getRadius();

    default int transformNormal(Vector3f mta, int normal) {
        return normal;
    }
}
