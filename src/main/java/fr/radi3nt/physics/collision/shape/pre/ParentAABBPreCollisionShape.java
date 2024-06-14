package fr.radi3nt.physics.collision.shape.pre;

import fr.radi3nt.maths.aabb.AxisMapping;
import fr.radi3nt.maths.aabb.SetAABB;
import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.detection.broad.aabb.CachingAABB;

public class ParentAABBPreCollisionShape implements PreCollisionShape {

    private final PreCollisionShape aabbPreCollisionShape;
    private final float added;

    private final CachingAABB enclosingCachedAabb;

    public ParentAABBPreCollisionShape(PreCollisionShape aabbPreCollisionShape, float added) {
        this.aabbPreCollisionShape = aabbPreCollisionShape;
        this.added = added;
        enclosingCachedAabb = new CachingAABB() {

            private final CachingAABB aabb = aabbPreCollisionShape.getAABB();

            @Override
            public void prepare(Vector3f position, Quaternion rotation) {
                aabb.prepare(position, rotation);
            }

            @Override
            public void release() {
                aabb.release();
            }

            @Override
            public void toSetAABB(SetAABB aabb, Vector3f position, Quaternion rotation) {
                this.aabb.toSetAABB(aabb, position, rotation);
                aabb.extend(added);
            }

            @Override
            public AxisMapping getxMapping() {
                throw new UnsupportedOperationException();
            }

            @Override
            public AxisMapping getxMapping(AxisMapping mapping) {
                throw new UnsupportedOperationException();
            }

            @Override
            public AxisMapping getyMapping() {
                throw new UnsupportedOperationException();
            }

            @Override
            public AxisMapping getyMapping(AxisMapping mapping) {
                throw new UnsupportedOperationException();
            }

            @Override
            public AxisMapping getzMapping() {
                throw new UnsupportedOperationException();
            }

            @Override
            public AxisMapping getzMapping(AxisMapping mapping) {
                throw new UnsupportedOperationException();
            }
        };
    }

    @Override
    public float getRadius() {
        return aabbPreCollisionShape.getRadius()+added;
    }

    @Override
    public Vector3f getSphereOffset() {
        return aabbPreCollisionShape.getSphereOffset();
    }

    @Override
    public CachingAABB getAABB() {
        return enclosingCachedAabb;
    }
}
