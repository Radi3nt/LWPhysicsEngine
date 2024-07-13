package fr.radi3nt.physics.collision.detection.broad.aabb.aabb;

import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.mapping.AxisMapping;
import fr.radi3nt.physics.collision.detection.narrow.processed.ProcessedShape;

public interface AABB extends ProcessedShape {

    AxisMapping getxMapping();
    AxisMapping getxMapping(AxisMapping mapping);
    AxisMapping getyMapping();
    AxisMapping getyMapping(AxisMapping mapping);
    AxisMapping getzMapping();
    AxisMapping getzMapping(AxisMapping mapping);


}
