package fr.radi3nt.physics.collision.shape;

import fr.radi3nt.physics.collision.detection.broad.pre.PreCollisionHolder;
import fr.radi3nt.physics.core.state.RigidBody;

public interface CollisionShapeProvider extends PreCollisionHolder {

    DuoCollisionShape[] getCollisionShapes(RigidBody other);
    boolean isEmpty();

}
