package fr.radi3nt.physics.collision.detection.broad.aabb.aabb;

import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.AABB;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.mapping.AxisMapping;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.SetAABB;
import fr.radi3nt.physics.core.TransformedObject;

public interface CachingAABB extends AABB {

    void prepare(TransformedObject transformedObject);
    void release();

    void toSetAABB(SetAABB aabb, TransformedObject transformedObject);

    static CachingAABB from(AABB aabb) {
        return new CachingAABB() {
            @Override
            public void prepare(TransformedObject transformedObject) {

            }

            @Override
            public void release() {

            }

            @Override
            public void toSetAABB(SetAABB otherAabb, TransformedObject transformedObject) {
                otherAabb.copy(aabb);
            }

            @Override
            public AxisMapping getxMapping() {
                return aabb.getxMapping();
            }

            @Override
            public AxisMapping getxMapping(AxisMapping mapping) {
                return aabb.getxMapping(mapping);
            }

            @Override
            public AxisMapping getyMapping() {
                return aabb.getyMapping();
            }

            @Override
            public AxisMapping getyMapping(AxisMapping mapping) {
                return aabb.getyMapping(mapping);
            }

            @Override
            public AxisMapping getzMapping() {
                return aabb.getzMapping();
            }

            @Override
            public AxisMapping getzMapping(AxisMapping mapping) {
                return aabb.getzMapping(mapping);
            }
        };
    }
}
