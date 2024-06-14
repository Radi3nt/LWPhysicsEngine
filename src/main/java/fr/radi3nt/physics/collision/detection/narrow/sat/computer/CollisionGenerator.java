package fr.radi3nt.physics.collision.detection.narrow.sat.computer;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;

public interface CollisionGenerator {

    CollisionResult test(GeneratedContactPair contactPair);

}
