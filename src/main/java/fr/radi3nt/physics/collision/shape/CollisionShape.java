package fr.radi3nt.physics.collision.shape;

import fr.radi3nt.physics.collision.shape.pre.PreCollisionHolder;
import fr.radi3nt.physics.collision.shape.pre.PreCollisionShape;

import java.util.Optional;

public interface CollisionShape extends PreCollisionHolder {

    PreCollisionShape getPreCollisionShape();

}
