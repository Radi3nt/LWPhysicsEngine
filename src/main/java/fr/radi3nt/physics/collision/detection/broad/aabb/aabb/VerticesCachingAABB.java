package fr.radi3nt.physics.collision.detection.broad.aabb.aabb;

import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.mapping.AxisMapping;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.SetAABB;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.maths.pool.ObjectPool;
import fr.radi3nt.maths.pool.Vector3fPool;
import fr.radi3nt.physics.core.TransformedObject;

public class VerticesCachingAABB implements CachingAABB {

    private static final ObjectPool<Vector3f> VECTOR_3_F_POOL = new Vector3fPool();

    private final Vector3f[] originalVertices;
    private final Vector3f[] cachedTransformedVertices;

    public VerticesCachingAABB(Vector3f[] originalVertices) {
        this.originalVertices = originalVertices;
        cachedTransformedVertices = new Vector3f[originalVertices.length];
    }

    @Override
    public void prepare(TransformedObject transformedObject) {
        for (int i = 0; i < cachedTransformedVertices.length; i++) {
            Vector3f currentVertex = cachedTransformedVertices[i] = VECTOR_3_F_POOL.borrow();
            currentVertex.copy(transformedObject.toWorldSpace(originalVertices[i]));
        }
    }

    @Override
    public void release() {
        for (Vector3f cachedTransformedVertex : cachedTransformedVertices) {
            VECTOR_3_F_POOL.free(cachedTransformedVertex);
        }
    }

    @Override
    public void toSetAABB(SetAABB aabb, TransformedObject transformedObject) {
        float maxX = -Float.MAX_VALUE;
        float minX = Float.MAX_VALUE;

        float maxY = -Float.MAX_VALUE;
        float minY = Float.MAX_VALUE;

        float maxZ = -Float.MAX_VALUE;
        float minZ = Float.MAX_VALUE;

        Vector3f currentVertex = new SimpleVector3f();
        for (Vector3f originalVertex : originalVertices) {
            currentVertex.copy(transformedObject.toWorldSpace(originalVertex));

            maxX = Math.max(maxX, currentVertex.getX());
            minX = Math.min(minX, currentVertex.getX());

            maxY = Math.max(maxY, currentVertex.getY());
            minY = Math.min(minY, currentVertex.getY());

            maxZ = Math.max(maxZ, currentVertex.getZ());
            minZ = Math.min(minZ, currentVertex.getZ());
        }

        aabb.copy(minX, maxX, minY, maxY, minZ, maxZ);
    }

    public AxisMapping getxMapping() {
        return getxMapping(new AxisMapping(0, 0));
    }

    @Override
    public AxisMapping getxMapping(AxisMapping mapping) {
        float maxX = -Float.MAX_VALUE;
        float minX = Float.MAX_VALUE;

        for (Vector3f currentVertex : cachedTransformedVertices) {
            maxX = Math.max(maxX, currentVertex.getX());
            minX = Math.min(minX, currentVertex.getX());
        }

        mapping.set(minX, maxX);
        return mapping;
    }

    public AxisMapping getyMapping() {
        return getyMapping(new AxisMapping(0, 0));
    }

    @Override
    public AxisMapping getyMapping(AxisMapping mapping) {
        float maxY = -Float.MAX_VALUE;
        float minY = Float.MAX_VALUE;

        for (Vector3f currentVertex : cachedTransformedVertices) {
            maxY = Math.max(maxY, currentVertex.getY());
            minY = Math.min(minY, currentVertex.getY());
        }
        mapping.set(minY, maxY);
        return mapping;
    }

    public AxisMapping getzMapping() {
        return getzMapping(new AxisMapping(0, 0));
    }

    @Override
    public AxisMapping getzMapping(AxisMapping mapping) {
        float maxZ = -Float.MAX_VALUE;
        float minZ = Float.MAX_VALUE;

        for (Vector3f currentVertex : cachedTransformedVertices) {
            maxZ = Math.max(maxZ, currentVertex.getZ());
            minZ = Math.min(minZ, currentVertex.getZ());
        }
        mapping.set(minZ, maxZ);
        return mapping;
    }
}
