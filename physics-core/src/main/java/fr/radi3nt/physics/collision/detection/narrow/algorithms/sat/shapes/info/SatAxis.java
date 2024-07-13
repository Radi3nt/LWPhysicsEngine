package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.info;

import fr.radi3nt.maths.components.vectors.Vector3f;

public interface SatAxis {

    Vector3f getAxis();
    SatFace[] getCorrespondingPlanes();
}
