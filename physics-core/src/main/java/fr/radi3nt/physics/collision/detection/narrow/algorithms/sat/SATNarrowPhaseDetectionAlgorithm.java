package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.contact.cache.PersistentManifoldCache;
import fr.radi3nt.physics.collision.contact.manifold.ManifoldPoint;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.collision.detection.narrow.NarrowPhaseDetectionAlgorithm;
import fr.radi3nt.physics.collision.detection.narrow.manifold.RegularManifoldComputer;
import fr.radi3nt.physics.collision.detection.narrow.processed.ProcessedShapeProvider;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.computer.NormalSatCollisionGenerator;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.SatProcessedShape;
import fr.radi3nt.physics.collision.shape.shapes.CollisionShape;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.Collection;
import java.util.Collections;
import java.util.List;

public class SATNarrowPhaseDetectionAlgorithm implements NarrowPhaseDetectionAlgorithm {

    private final ProcessedShapeProvider<SatProcessedShape> processedShapeProvider;
    private final NormalSatCollisionGenerator generator = new NormalSatCollisionGenerator();
    private final RegularManifoldComputer manifoldComputer;

    public SATNarrowPhaseDetectionAlgorithm(ProcessedShapeProvider<SatProcessedShape> processedShapeProvider) {
        this.processedShapeProvider = processedShapeProvider;
        manifoldComputer = new RegularManifoldComputer();
    }

    @Override
    public PersistentManifold buildManifolds(PersistentManifoldCache manifoldCache, GeneratedContactPair<RigidBody> pair, long currentStep) {
        Collection<ManifoldPoint> collision = buildManifoldPoints(pair);
        if (collision==null)
            collision = Collections.emptyList();

        return manifoldComputer.compute(manifoldCache, pair, collision, currentStep);
    }

    @Override
    public List<ManifoldPoint> buildManifoldPoints(GeneratedContactPair<?> pair) {
        SatProcessedShape shapeA = processedShapeProvider.getShape(pair.shapeA, pair.objectA);
        SatProcessedShape shapeB = processedShapeProvider.getShape(pair.shapeB, pair.objectB);

        List<ManifoldPoint> collision = generator.test(pair, shapeA, shapeB);

        if (collision.isEmpty()) {
            return null;
        }
        return collision;
    }

    @Override
    public boolean isSupported(CollisionShape collisionShape) {
        return processedShapeProvider.isSupported(collisionShape);
    }
}
