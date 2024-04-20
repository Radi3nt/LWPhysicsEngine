package fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.normal;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.contact.manifold.ManifoldPoint;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.CollisionResult;
import fr.radi3nt.physics.collision.detection.narrow.sat.computer.part.ShapedPair;

import java.util.List;

public class NormalCollisionResult implements CollisionResult {

    private final NormalReuseManifoldPointBuilder pointBuilder;

    private ShapedPair shapedPair;

    private Vector3f worldSpaceNormal;
    private int normal;
    private float overlap;

    public NormalCollisionResult(NormalReuseManifoldPointBuilder pointBuilder) {
        this.pointBuilder = pointBuilder;
    }

    public void set(ShapedPair shapedPair, Vector3f worldSpaceNormal, int normal, float overlap) {
        this.shapedPair = shapedPair;
        this.worldSpaceNormal = worldSpaceNormal;
        this.normal = normal;
        this.overlap = overlap;
    }

    @Override
    public boolean isCollision() {
        return true;
    }

    @Override
    public float getOverlap() {
        return overlap;
    }

    @Override
    public Vector3f getWorldSpaceNormal() {
        return worldSpaceNormal;
    }

    @Override
    public Vector3f getWorldSpaceDirectedNormal() {
        return worldSpaceNormal.duplicate().mul(normal);
    }

    @Override
    public List<ManifoldPoint> createPoints() {
        return pointBuilder.computeManifolds(shapedPair, getWorldSpaceDirectedNormal(), getOverlap(), normal);
    }

    @Override
    public String toString() {
        return "NormalCollisionResult{" +
                "pointBuilder=" + pointBuilder +
                ", shapedPair=" + shapedPair +
                ", worldSpaceNormal=" + worldSpaceNormal +
                ", normal=" + normal +
                ", overlap=" + overlap +
                '}';
    }
}
