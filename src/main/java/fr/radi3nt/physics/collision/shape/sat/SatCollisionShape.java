package fr.radi3nt.physics.collision.shape.sat;

import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.shape.CollisionShape;
import fr.radi3nt.physics.collision.shape.sat.shape.SatShapeObject;

public interface SatCollisionShape extends CollisionShape {

    SatShapeObject getShape();

}
