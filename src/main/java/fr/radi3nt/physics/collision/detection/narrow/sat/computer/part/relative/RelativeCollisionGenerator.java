package fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.relative;

import fr.radi3nt.physics.collision.detection.narrow.sat.computer.CollisionGenerator;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.CollisionDetector;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.ManifoldPointBuilder;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.relative.RelativeSatCollisionDetector;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.relative.RelativeReuseManifoldPointBuilder;

public class RelativeCollisionGenerator implements CollisionGenerator {

    private final RelativeSatCollisionDetector normalSatCollisionDetector = new RelativeSatCollisionDetector();
    private final RelativeReuseManifoldPointBuilder manifoldPointBuilder;

    public RelativeCollisionGenerator(float penetrationEpsilon) {
        manifoldPointBuilder = new RelativeReuseManifoldPointBuilder(normalSatCollisionDetector, penetrationEpsilon);
    }


    @Override
    public CollisionDetector getDetector() {
        return normalSatCollisionDetector;
    }

    @Override
    public ManifoldPointBuilder getPointBuilder() {
        return manifoldPointBuilder;
    }
}
