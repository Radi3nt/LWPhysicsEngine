package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.simple;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.pool.ObjectPool;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.computer.detector.NormalSatCollisionDetector;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.info.SatAxis;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.info.SatEdge;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.projection.SatProjectionProvider;
import fr.radi3nt.physics.core.TransformedObject;

public interface SatShape {

    SatAxis[] getSatAxis();
    SatEdge[] getSatEdges();

    boolean canUseEdgesAsMinimum();
    SatProjectionProvider getSatProjectionProvider(TransformedObject object, ObjectPool<Vector3f> pool);

    default int transformNormal(Vector3f mta, int normal, boolean isA, NormalSatCollisionDetector.SatContactType contactType) {
        return normal;
    }

}
