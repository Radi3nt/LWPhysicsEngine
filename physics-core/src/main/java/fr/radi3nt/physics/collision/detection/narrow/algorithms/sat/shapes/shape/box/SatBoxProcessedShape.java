package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.box;

import fr.radi3nt.maths.components.advanced.quaternions.ComponentsQuaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.maths.pool.ObjectPool;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.ClipPlane;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.ClipPlanes;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.Edge;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.info.*;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.projection.AxisProjectionProvider;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.projection.SatProjectionProvider;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.projection.VerticesProjectionProvider;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.SatProcessedShape;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.StoredSatProcessedShape;
import fr.radi3nt.physics.core.TransformedObject;

import static java.lang.Math.abs;

public class SatBoxProcessedShape extends StoredSatProcessedShape implements SatProcessedShape {

    private final TransformedObject transformedObject;
    private final AxisProjectionProvider provider;
    private final Vector3f size;
    private SatEdge[] edges;

    public SatBoxProcessedShape(TransformedObject transformedObject, Vector3f size) {
        this.transformedObject = transformedObject;
        this.size = size;
        provider = new AxisProjectionProvider(size, new SimpleVector3f(), ComponentsQuaternion.zero(), transformedObject);;
    }

    public SatBoxProcessedShape compute() {
        computeVertices();
        computeAxisArray();
        computeClipEdges();
        computeClipPlanes();
        return this;
    }

    private void computeClipPlanes() {
        clipPlanes = new ClipPlanes(
                new ClipPlane(new SimpleVector3f(1, 0, 0), vertices[1]),
                new ClipPlane(new SimpleVector3f(0, 1, 0), vertices[1]),
                new ClipPlane(new SimpleVector3f(0, 0, 1), vertices[1]),
                new ClipPlane(new SimpleVector3f(-1, 0, 0), vertices[0]),
                new ClipPlane(new SimpleVector3f(0, -1, 0), vertices[0]),
                new ClipPlane(new SimpleVector3f(0, 0, -1), vertices[0])
        );
    }

    private void computeClipEdges() {
        clipEdges = new Edge[] {
                new Edge(vertices[0], vertices[6]),
                new Edge(vertices[0], vertices[4]),
                new Edge(vertices[0], vertices[2]),

                new Edge(vertices[1], vertices[7]),
                new Edge(vertices[1], vertices[5]),
                new Edge(vertices[1], vertices[3]),

                new Edge(vertices[6], vertices[7]),
                new Edge(vertices[7], vertices[2]),

                new Edge(vertices[5], vertices[4]),
                new Edge(vertices[4], vertices[3]),

                new Edge(vertices[5], vertices[6]),
                new Edge(vertices[3], vertices[2])
        };
    }

    private void computeAxisArray() {
        axis = new Vector3f[]{
                vertices[2].duplicate().sub(vertices[0]).normalize(),
                vertices[4].duplicate().sub(vertices[0]).normalize(),
                vertices[6].duplicate().sub(vertices[0]).normalize(),
        };
    }

    protected void computeVertices() {
        Vector3f min = size.duplicate().negate();
        Vector3f max = size.duplicate();
        vertices = new Vector3f[]{
                min.duplicate(),
                max.duplicate(),
                new SimpleVector3f(max.getX(), min.getY(), min.getZ()),
                new SimpleVector3f(max.getX(), max.getY(), min.getZ()),
                new SimpleVector3f(min.getX(), max.getY(), min.getZ()),
                new SimpleVector3f(min.getX(), max.getY(), max.getZ()),
                new SimpleVector3f(min.getX(), min.getY(), max.getZ()),
                new SimpleVector3f(max.getX(), min.getY(), max.getZ())
        };
    }

    @Override
    public SatProjectionProvider getSatProjectionProvider(TransformedObject object, ObjectPool<Vector3f> pool) {
        return provider;
    }

    @Override
    public SatAxis[] getSatAxis() {
        SatFace[] faces = getSatFaces();
        return new SatAxis[] {
                new SetSatAxis(axis[0], new SatFace[]{ //z-axis
                        faces[0],
                        faces[1],
                }),
                new SetSatAxis(axis[1], new SatFace[]{ //y-axis
                        faces[2],
                        faces[3],
                }),
                new SetSatAxis(axis[2], new SatFace[]{ //x-axis
                        faces[4],
                        faces[5],
                })
        };
    }

    @Override
    public SatFace[] getSatFaces() {
        return new SatFace[] {
                new ComputingSatFace(clipPlanes.getClipPlanes()[0], clipPlanes.getClipPlanes(), getClipEdges()),
                new ComputingSatFace(clipPlanes.getClipPlanes()[3], clipPlanes.getClipPlanes(), getClipEdges()),

                new ComputingSatFace(clipPlanes.getClipPlanes()[1], clipPlanes.getClipPlanes(), getClipEdges()),
                new ComputingSatFace(clipPlanes.getClipPlanes()[4], clipPlanes.getClipPlanes(), getClipEdges()),

                new ComputingSatFace(clipPlanes.getClipPlanes()[2], clipPlanes.getClipPlanes(), getClipEdges()),
                new ComputingSatFace(clipPlanes.getClipPlanes()[5], clipPlanes.getClipPlanes(), getClipEdges())
        };
    }

    @Override
    public SatEdge[] getSatEdges() {
        if (edges ==null) {
             Vector3f[] edgesAxis = computeEdgesAxis();
             edges = new SatEdge[] {
                     new ComputingSatEdge(axis[0], getClipEdges(), edgesAxis),
                     new ComputingSatEdge(axis[1], getClipEdges(), edgesAxis),
                     new ComputingSatEdge(axis[2], getClipEdges(), edgesAxis),
             };
        }
        return edges;
    }



    @Override
    public Vector3f[] getEdges() {
        return axis;
    }

    public Vector3f getSize() {
        return size;
    }
}
