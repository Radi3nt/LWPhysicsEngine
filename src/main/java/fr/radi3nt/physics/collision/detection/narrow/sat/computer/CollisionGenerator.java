package fr.radi3nt.physics.collision.detection.narrow.sat.computer;

import fr.radi3nt.physics.collision.contact.ContactPair;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.CollisionDetector;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.ManifoldPointBuilder;

public interface CollisionGenerator {

    CollisionResult test(ContactPair contactPair);

}
