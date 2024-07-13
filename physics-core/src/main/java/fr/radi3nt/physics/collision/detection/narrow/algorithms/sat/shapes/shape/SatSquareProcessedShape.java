package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape;

import fr.radi3nt.maths.components.vectors.Vector2f;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.ClipPlane;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.ClipPlanes;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.Edge;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.info.*;

public class SatSquareProcessedShape extends StoredSatProcessedShape implements SatProcessedShape {

    private final Vector2f size;

    public SatSquareProcessedShape(Vector2f size) {
        this.size = size;
    }

    public SatSquareProcessedShape compute() {
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
                new ClipPlane(new SimpleVector3f(-1, 0, 0), vertices[0]),
                new ClipPlane(new SimpleVector3f(0, -1, 0), vertices[0])
        );
    }

    private void computeClipEdges() {
        clipEdges = new Edge[] {
                new Edge(vertices[0], vertices[2]),
                new Edge(vertices[0], vertices[3]),
                new Edge(vertices[1], vertices[2]),
                new Edge(vertices[1], vertices[3]),
        };
    }

    private void computeAxisArray() {
        axis = new Vector3f[] {
                vertices[3].duplicate().sub(vertices[0]).normalize(),
                vertices[0].duplicate().sub(vertices[2]).normalize(),
        };
    }

    protected void computeVertices() {
        Vector2f min = size.clone().mul(-1);
        Vector2f max = size.clone();
        vertices = new Vector3f[]{
                new SimpleVector3f(min.getX(), min.getY(), 0),
                new SimpleVector3f(max.getX(), max.getY(), 0),
                new SimpleVector3f(min.getX(), max.getY(), 0),
                new SimpleVector3f(max.getX(), min.getY(), 0)
        };
    }

    @Override
    public SatAxis[] getSatAxis() {
        return new SatAxis[] {
                new SetSatAxis(axis[0], new SatFace[]{
                        getSatFaces()[0],
                        getSatFaces()[1],
                }),
                new SetSatAxis(axis[1], new SatFace[]{
                        getSatFaces()[2],
                        getSatFaces()[3],
                })
        };
    }

    @Override
    public SatFace[] getSatFaces() {
        return new SatFace[] {
                new ComputingSatFace(clipPlanes.getClipPlanes()[0], clipPlanes.getClipPlanes(), getClipEdges()),
                new ComputingSatFace(clipPlanes.getClipPlanes()[2], clipPlanes.getClipPlanes(), getClipEdges()),

                new ComputingSatFace(clipPlanes.getClipPlanes()[1], clipPlanes.getClipPlanes(), getClipEdges()),
                new ComputingSatFace(clipPlanes.getClipPlanes()[3], clipPlanes.getClipPlanes(), getClipEdges()),
        };
    }

    @Override
    public SatEdge[] getSatEdges() {
        return new SatEdge[0];
        /*
        Vector3f[] edgesAxis = computeEdgesAxis();
        return new SatEdge[] {
                new ComputingSatEdge(axis[0], getClipEdges(), edgesAxis),
                new ComputingSatEdge(axis[1], getClipEdges(), edgesAxis),
        };

         */
    }

    @Override
    public Vector3f[] getEdges() {
        return new Vector3f[0];
    }

    public Vector2f getSize() {
        return size;
    }
}
