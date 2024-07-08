package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.maths.pool.ObjectPool;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.CachingAABB;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.VerticesCachingAABB;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.projection.SatProjectionProvider;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.ClipPlanes;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.Edge;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.projection.VerticesProjectionProvider;
import fr.radi3nt.physics.core.TransformedObject;

public abstract class StoredSatProcessedShape implements SatProcessedShape {

    protected Vector3f[] axis;
    protected Vector3f[] vertices;
    protected ClipPlanes clipPlanes;
    protected Edge[] clipEdges;

    @Override
    public Vector3f[] getAxis() {
        return axis;
    }

    @Override
    public ClipPlanes getClipPlanes() {
        return clipPlanes;
    }

    @Override
    public Edge[] getClipEdges() {
        return clipEdges;
    }

    @Override
    public SatProjectionProvider getSatProjectionProvider(TransformedObject object, ObjectPool<Vector3f> pool) {
        return new VerticesProjectionProvider(getTransformedArray(vertices, object));
    }

    public static Vector3f[] getTransformedArray(Vector3f[] originalArray, TransformedObject object) {
        Vector3f[] shapeAxisArray = new Vector3f[originalArray.length];
        for (int i = 0; i < originalArray.length; i++) {
            shapeAxisArray[i] = object.toWorldSpace(originalArray[i]);
        }
        return shapeAxisArray;
    }

    @Override
    public boolean canUseEdgesAsMinimum() {
        return true;
    }

    protected Vector3f[] computeEdgesAxis() {
        Edge[] edges = getClipEdges();
        Vector3f[] axis = new SimpleVector3f[edges.length];

        for (int i = 0; i < edges.length; i++) {
            Edge edge = edges[i];
            Vector3f currentAxis = edge.getVertex2().duplicate().sub(edge.getVertex1()).normalize();
            axis[i] = currentAxis;
        }

        return axis;
    }
}
