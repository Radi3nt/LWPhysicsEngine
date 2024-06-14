package fr.radi3nt.physics.collision.detection.narrow.sat.computer.part;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.shape.sat.shape.SatShapeObject;

public interface CollisionDetector {

    boolean testCollision(GeneratedContactPair pair, SatShapeObject sa, SatShapeObject sb);

    Vector3f getMinimumTranslationAxis();

    float getCurrentOverlap();

    int getOverlapNormal();
}
