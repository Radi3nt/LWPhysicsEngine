package fr.radi3nt.physics.collision.detection.gen.provider;

import fr.radi3nt.maths.aabb.AxisMapping;
import fr.radi3nt.physics.collision.CollisionShapeGroup;
import fr.radi3nt.physics.core.TransformedObject;
import fr.radi3nt.physics.core.state.RigidBody;

public interface OneDimensionProvider {

    AxisMapping map(TransformedObject body, CollisionShapeGroup shapeGroup);

}
