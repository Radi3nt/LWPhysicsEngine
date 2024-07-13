package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.info;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.Edge;

public interface SatEdge {

    Vector3f getAxis();
    Edge[] getEdges();

}
