package fr.radi3nt.physics.collision.detection.broad.aabb.aabb;

import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.mapping.AxisMapping;

public class ExpendingAABB extends DynamicAABB {

    private final AABB aabb;
    private final float extended;

    public ExpendingAABB(AABB aabb, float extended) {
        this.aabb = aabb;
        this.extended = extended;
    }

    @Override
    public AxisMapping getxMapping(AxisMapping mapping) {
        aabb.getxMapping(mapping);
        mapping.set(mapping.getMin()-extended, mapping.getMax()+extended);
        return mapping;
    }

    @Override
    public AxisMapping getyMapping(AxisMapping mapping) {
        aabb.getyMapping(mapping);
        mapping.set(mapping.getMin()-extended, mapping.getMax()+extended);
        return mapping;
    }

    @Override
    public AxisMapping getzMapping(AxisMapping mapping) {
        aabb.getzMapping(mapping);
        mapping.set(mapping.getMin()-extended, mapping.getMax()+extended);
        return mapping;
    }
}
