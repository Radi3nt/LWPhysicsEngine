package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.info.SatFace;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.simple.SatShape;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.ClipPlanes;
import fr.radi3nt.physics.collision.detection.narrow.processed.ProcessedShape;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.Edge;

public interface SatProcessedShape extends ProcessedShape, SatShape {

    @Deprecated
    Vector3f[] getAxis();
    @Deprecated
    Vector3f[] getEdges();

    SatFace[] getSatFaces();
    ClipPlanes getClipPlanes();
    Edge[] getClipEdges();

}
