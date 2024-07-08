package fr.radi3nt.physics.collision.detection.generators.provider;

import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.mapping.AxisMapping;
import fr.radi3nt.physics.collision.detection.broad.pre.PreCollisionShape;
import fr.radi3nt.physics.core.TransformedObject;

public interface OneDimensionProvider {

    AxisMapping map(TransformedObject body, PreCollisionShape shapeGroup);

}
