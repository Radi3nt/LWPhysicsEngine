package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.info;

import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.ClipPlane;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.Edge;

public interface SatFace {

    ClipPlane getClipPlane();
    ClipPlane[] getNeighbourPlanes();
    Edge[] getFaceEdge();
}
