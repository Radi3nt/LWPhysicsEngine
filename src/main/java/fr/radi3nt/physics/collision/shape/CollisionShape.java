package fr.radi3nt.physics.collision.shape;

import fr.radi3nt.physics.collision.shape.pre.PreCollisionHolder;
import fr.radi3nt.physics.collision.shape.pre.PreCollisionShape;

public interface CollisionShape extends PreCollisionHolder {

    PreCollisionShape getPreCollisionShape();

}
