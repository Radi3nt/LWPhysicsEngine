package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.simple;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.pool.ObjectPool;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.Edge;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.info.*;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.projection.SatProjectionProvider;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.projection.VerticesProjectionProvider;
import fr.radi3nt.physics.core.TransformedObject;

public class RaySatShape implements SatShape {

    private final Vector3f pointOnRay;
    private final Vector3f ray;

    public RaySatShape(Vector3f pointOnRay, Vector3f ray) {
        this.pointOnRay = pointOnRay;
        this.ray = ray;
    }

    @Override
    public SatAxis[] getSatAxis() {
        return new SatAxis[] {new SetSatAxis(ray.duplicate(), new SatFace[0])};
    }

    @Override
    public SatEdge[] getSatEdges() {
        return new SatEdge[] {new SetSatEdge(ray.duplicate(), new Edge[0])};
    }

    @Override
    public boolean canUseEdgesAsMinimum() {
        return true;
    }

    @Override
    public SatProjectionProvider getSatProjectionProvider(TransformedObject object, ObjectPool<Vector3f> pool) {
        return new VerticesProjectionProvider(new Vector3f[]{pointOnRay.duplicate()});
    }

    public Vector3f getPointOnRay() {
        return pointOnRay.duplicate();
    }

    public Vector3f getRay() {
        return ray.duplicate();
    }
}
