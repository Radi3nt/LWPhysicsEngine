package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.computer;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.contact.manifold.ManifoldPoint;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.computer.detector.NormalSatCollisionDetector;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.computer.manifold.NormalManifoldPointBuilder;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.SatProcessedShape;

import java.util.Collections;
import java.util.List;

public class NormalSatCollisionGenerator implements SatCollisionGenerator {

    private final NormalManifoldPointBuilder manifoldPointBuilder;

    public NormalSatCollisionGenerator() {
        manifoldPointBuilder = new NormalManifoldPointBuilder();
    }

    @Override
    public List<ManifoldPoint> test(GeneratedContactPair<?> contactPair, SatProcessedShape sa, SatProcessedShape sb) {
        NormalSatCollisionDetector normalSatCollisionDetector = new NormalSatCollisionDetector();
        boolean collision = normalSatCollisionDetector.testCollision(contactPair.objectA, contactPair.objectB, sa, sb);
        if (!collision)
            return Collections.emptyList();
        return manifoldPointBuilder.createContactPoints(contactPair, sa, sb, normalSatCollisionDetector.getResultInfo());
    }
}
