package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.computer;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.projection.SatProjectedObject;

public class NormalSatPointDetector {

    public float getPenetration(Vector3f point, Vector3f normal, Vector3f[] shape) {
        SatProjectedObject p1 = new SatProjectedObject(normal.dot(point), normal.dot(point));
        SatProjectedObject p2 = SatProjectedObject.project(shape, normal);

        return p2.contains(p1) ? SatProjectedObject.getNormalPointingContainmentOverlap(p1, p2) : 0;
    }

}
