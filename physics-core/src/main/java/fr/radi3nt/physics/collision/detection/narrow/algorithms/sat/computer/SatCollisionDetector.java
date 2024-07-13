package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.computer;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.computer.NormalSatCollisionDetector;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.SatProcessedShape;

public interface SatCollisionDetector {

    boolean testCollision(GeneratedContactPair pair, SatProcessedShape sa, SatProcessedShape sb);
    NormalSatCollisionDetector.ResultInfo getResultInfo();

}
