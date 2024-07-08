package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.pool.ObjectPool;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.computer.NormalSatCollisionDetector;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.ClipPlanes;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.info.SatAxis;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.info.SatEdge;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.info.SatFace;
import fr.radi3nt.physics.collision.detection.narrow.processed.ProcessedShape;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.projection.SatProjectionProvider;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.Edge;
import fr.radi3nt.physics.core.TransformedObject;

public interface SatProcessedShape extends ProcessedShape {

    SatAxis[] getSatAxis();
    SatFace[] getSatFaces();
    SatEdge[] getSatEdges();

    Vector3f[] getAxis();
    Vector3f[] getEdges();

    ClipPlanes getClipPlanes();
    Edge[] getClipEdges();

    boolean canUseEdgesAsMinimum();

    SatProjectionProvider getSatProjectionProvider(TransformedObject object, ObjectPool<Vector3f> pool);

    default int transformNormal(Vector3f mta, int normal, boolean isA, NormalSatCollisionDetector.SatContactType contactType) {
        return normal;
    }
}
