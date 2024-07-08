package fr.radi3nt.physics.collision.detection.broad.aabb.aabb;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.mapping.AxisMapping;
import fr.radi3nt.physics.core.TransformedObject;

public class VerticesAABB implements AABB {

    private final Vector3f[] transformedVertices;

    public VerticesAABB(Vector3f[] vertices) {
        transformedVertices = vertices;
    }

    public static VerticesAABB from(TransformedObject transformedObject, Vector3f... vertices) {
        Vector3f[] transformedVertices = new Vector3f[vertices.length];
        for (int i = 0, verticesLength = vertices.length; i < verticesLength; i++) {
            Vector3f vertex = vertices[i];
            transformedVertices[i] = transformedObject.toWorldSpace(vertex);
        }
        return new VerticesAABB(transformedVertices);
    }

    public AxisMapping getxMapping() {
        float maxX = -Float.MAX_VALUE;
        float minX = Float.MAX_VALUE;

        for (Vector3f currentVertex : transformedVertices) {
            maxX = Math.max(maxX, currentVertex.getX());
            minX = Math.min(minX, currentVertex.getX());
        }
        return new AxisMapping(minX, maxX);
    }

    @Override
    public AxisMapping getxMapping(AxisMapping mapping) {
        float maxX = -Float.MAX_VALUE;
        float minX = Float.MAX_VALUE;
        for (Vector3f currentVertex : transformedVertices) {
            maxX = Math.max(maxX, currentVertex.getX());
            minX = Math.min(minX, currentVertex.getX());
        }
        mapping.set(minX, maxX);
        return mapping;
    }

    public AxisMapping getyMapping() {
        float maxY = -Float.MAX_VALUE;
        float minY = Float.MAX_VALUE;

        for (Vector3f currentVertex : transformedVertices) {
            maxY = Math.max(maxY, currentVertex.getY());
            minY = Math.min(minY, currentVertex.getY());
        }
        return new AxisMapping(minY, maxY);
    }

    @Override
    public AxisMapping getyMapping(AxisMapping mapping) {
        float maxY = -Float.MAX_VALUE;
        float minY = Float.MAX_VALUE;

        for (Vector3f currentVertex : transformedVertices) {
            maxY = Math.max(maxY, currentVertex.getY());
            minY = Math.min(minY, currentVertex.getY());
        }
        mapping.set(minY, maxY);
        return mapping;
    }

    public AxisMapping getzMapping() {
        float maxZ = -Float.MAX_VALUE;
        float minZ = Float.MAX_VALUE;

        for (Vector3f currentVertex : transformedVertices) {
            maxZ = Math.max(maxZ, currentVertex.getZ());
            minZ = Math.min(minZ, currentVertex.getZ());
        }
        return new AxisMapping(minZ, maxZ);
    }

    @Override
    public AxisMapping getzMapping(AxisMapping mapping) {
        float maxZ = -Float.MAX_VALUE;
        float minZ = Float.MAX_VALUE;

        for (Vector3f currentVertex : transformedVertices) {
            maxZ = Math.max(maxZ, currentVertex.getZ());
            minZ = Math.min(minZ, currentVertex.getZ());
        }
        mapping.set(minZ, maxZ);
        return mapping;
    }

}
