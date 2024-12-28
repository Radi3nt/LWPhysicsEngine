package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.computer;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.contact.manifold.ManifoldPoint;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.SatProcessedShape;

import java.util.List;

public interface SatCollisionGenerator {

    List<ManifoldPoint> test(GeneratedContactPair<?> contactPair, SatProcessedShape shapeA, SatProcessedShape shapeB);

}
