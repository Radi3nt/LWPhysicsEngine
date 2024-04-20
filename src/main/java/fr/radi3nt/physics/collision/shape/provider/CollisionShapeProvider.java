package fr.radi3nt.physics.collision.shape.provider;

import fr.radi3nt.physics.collision.shape.CollisionShape;
import fr.radi3nt.physics.core.TransformedObject;
import fr.radi3nt.physics.core.state.RigidBody;

public interface CollisionShapeProvider {

    CollisionShape[] getCollisionShape(RigidBody other);

}
