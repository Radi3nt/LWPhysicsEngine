package fr.radi3nt.physics.collision.detection.broad.aabb.aabb;

import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.mapping.AxisMapping;

public abstract class DynamicAABB implements AABB {

    @Override
    public AxisMapping getxMapping() {
        return getxMapping(new AxisMapping(0, 0));
    }

    @Override
    public AxisMapping getyMapping() {
        return getyMapping(new AxisMapping(0, 0));
    }

    @Override
    public AxisMapping getzMapping() {
        return getzMapping(new AxisMapping(0, 0));
    }
}
