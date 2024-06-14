package fr.radi3nt.physics.collision.detection.broad.aabb;

import fr.radi3nt.maths.aabb.AABB;
import fr.radi3nt.maths.aabb.AxisMapping;
import fr.radi3nt.maths.aabb.SetAABB;
import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;

public interface CachingAABB extends AABB {

    void prepare(Vector3f position, Quaternion rotation);
    void release();

    void toSetAABB(SetAABB aabb, Vector3f position, Quaternion rotation);

    static CachingAABB from(AABB aabb) {
        return new CachingAABB() {
            @Override
            public void prepare(Vector3f position, Quaternion rotation) {

            }

            @Override
            public void release() {

            }

            @Override
            public void toSetAABB(SetAABB otherAabb, Vector3f position, Quaternion rotation) {
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
