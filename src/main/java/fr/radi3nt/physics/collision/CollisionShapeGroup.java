package fr.radi3nt.physics.collision;

import fr.radi3nt.physics.collision.shape.CollisionShape;
import fr.radi3nt.physics.collision.shape.pre.PreCollisionHolder;
import fr.radi3nt.physics.collision.shape.pre.PreCollisionShape;
import fr.radi3nt.physics.core.TransformedObject;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.Optional;

public interface CollisionShapeGroup extends PreCollisionHolder {

    CollisionShape[] getCollisionShape(RigidBody other);
    PreCollisionShape getPreCollisionShape();

}
