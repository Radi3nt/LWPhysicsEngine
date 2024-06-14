package fr.radi3nt.physics.collision.detection.broad.aabb;

import fr.radi3nt.maths.aabb.AxisMapping;
import fr.radi3nt.maths.aabb.SetAABB;
import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.maths.pool.ObjectPool;
import fr.radi3nt.maths.pool.Vector3fPool;

public class VerticesCachingAABB implements CachingAABB {

    private static final ObjectPool<Vector3f> VECTOR_3_F_POOL = new Vector3fPool();

    private final Vector3f[] originalVertices;
    private final Vector3f[] cachedTransformedVertices;

    public VerticesCachingAABB(Vector3f[] originalVertices) {
        this.originalVertices = originalVertices;
        cachedTransformedVertices = new Vector3f[originalVertices.length];
    }

    @Override
    public void prepare(Vector3f position, Quaternion rotation) {
        for (int i = 0; i < cachedTransformedVertices.length; i++) {
            Vector3f currentVertex = cachedTransformedVertices[i] = VECTOR_3_F_POOL.borrow();
            currentVertex.copy(originalVertices[i]);

            rotation.transform(currentVertex);
            currentVertex.add(position);
        }
    }

    @Override
    public void release() {
        for (Vector3f cachedTransformedVertex : cachedTransformedVertices) {
            VECTOR_3_F_POOL.free(cachedTransformedVertex);
        }
    }

    @Override
    public void toSetAABB(SetAABB aabb, Vector3f position, Quaternion rotation) {

        float maxX = -Float.MAX_VALUE;
        float minX = Float.MAX_VALUE;

        float maxY = -Float.MAX_VALUE;
        float minY = Float.MAX_VALUE;

        float maxZ = -Float.MAX_VALUE;
        float minZ = Float.MAX_VALUE;

        Vector3f currentVertex = new SimpleVector3f();
        for (Vector3f vertex : originalVertices) {
            currentVertex.copy(vertex);

            rotation.transform(currentVertex);
            currentVertex.add(position);

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
        float maxX = -Float.MAX_VALUE;
        float minX = Float.MAX_VALUE;

        Vector3f[] array = cachedTransformedVertices;
        for (Vector3f currentVertex : array) {
            maxX = Math.max(maxX, currentVertex.getX());
            minX = Math.min(minX, currentVertex.getX());
        }
        return new AxisMapping(minX, maxX);
    }

    @Override
    public AxisMapping getxMapping(AxisMapping mapping) {
        float maxX = -Float.MAX_VALUE;
        float minX = Float.MAX_VALUE;

        Vector3f[] array = cachedTransformedVertices;
        for (Vector3f currentVertex : array) {
            maxX = Math.max(maxX, currentVertex.getX());
            minX = Math.min(minX, currentVertex.getX());
        }

        mapping.set(minX, maxX);
        return mapping;
    }

    public AxisMapping getyMapping() {
        float maxY = -Float.MAX_VALUE;
        float minY = Float.MAX_VALUE;

        Vector3f[] array = cachedTransformedVertices;
        for (Vector3f currentVertex : array) {
            maxY = Math.max(maxY, currentVertex.getY());
            minY = Math.min(minY, currentVertex.getY());
        }
        return new AxisMapping(minY, maxY);
    }

    @Override
    public AxisMapping getyMapping(AxisMapping mapping) {
        float maxY = -Float.MAX_VALUE;
        float minY = Float.MAX_VALUE;

        Vector3f[] array = cachedTransformedVertices;
        for (Vector3f currentVertex : array) {
            maxY = Math.max(maxY, currentVertex.getY());
            minY = Math.min(minY, currentVertex.getY());
        }
        mapping.set(minY, maxY);
        return mapping;
    }

    public AxisMapping getzMapping() {
        float maxZ = -Float.MAX_VALUE;
        float minZ = Float.MAX_VALUE;

        Vector3f[] array = cachedTransformedVertices;
        for (Vector3f currentVertex : array) {
            maxZ = Math.max(maxZ, currentVertex.getZ());
            minZ = Math.min(minZ, currentVertex.getZ());
        }
        return new AxisMapping(minZ, maxZ);
    }

    @Override
    public AxisMapping getzMapping(AxisMapping mapping) {
        float maxZ = -Float.MAX_VALUE;
        float minZ = Float.MAX_VALUE;

        Vector3f[] array = cachedTransformedVertices;
        for (Vector3f currentVertex : array) {
            maxZ = Math.max(maxZ, currentVertex.getZ());
            minZ = Math.min(minZ, currentVertex.getZ());
        }
        mapping.set(minZ, maxZ);
        return mapping;
    }
}
