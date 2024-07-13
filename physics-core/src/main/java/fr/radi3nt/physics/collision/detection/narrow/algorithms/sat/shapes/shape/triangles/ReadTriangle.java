package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.triangles;

import fr.radi3nt.maths.components.vectors.Vector3f;

public interface ReadTriangle {

    Vector3f getVertex1();
    Vector3f getVertex2();
    Vector3f getVertex3();

    Vector3f getNormal();
}
