package fr.radi3nt.physics.collision.detection.narrow.sat;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.contact.cache.PersistentManifoldCache;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.collision.detection.narrow.NarrowPhaseDetectionAlgorithm;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.CollisionGenerator;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.CollisionResult;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.normal.NormalCollisionGenerator;

import java.util.Optional;

public class SATNarrowPhaseDetectionAlgorithm implements NarrowPhaseDetectionAlgorithm {

    private final CollisionGenerator generator = new NormalCollisionGenerator();
    private final SatManifoldComputer manifoldComputer;

    public SATNarrowPhaseDetectionAlgorithm() {
        manifoldComputer = new SatManifoldComputer();
    }

    @Override
    public Optional<PersistentManifold> buildManifolds(PersistentManifoldCache manifoldCache, GeneratedContactPair pair) {
        CollisionResult collision = generator.test(pair);

        if (!collision.isCollision()) {
            return Optional.empty();
        }

        return manifoldComputer.compute(manifoldCache, pair, collision);
    }
}
