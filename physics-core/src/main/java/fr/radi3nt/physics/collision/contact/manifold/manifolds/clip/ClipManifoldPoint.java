package fr.radi3nt.physics.collision.contact.manifold.manifolds.clip;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.contact.manifold.ManifoldPoint;
import fr.radi3nt.physics.collision.contact.manifold.PointIndex;
import fr.radi3nt.physics.core.TransformedObject;

import java.util.Objects;

public class ClipManifoldPoint extends ManifoldPoint {

    private final PointIndex pointIndex;

    public ClipManifoldPoint(Vector3f localSpaceAPoint, Vector3f localSpaceBPoint, Vector3f normal, PointIndex pointIndex) {
        super(localSpaceAPoint, localSpaceBPoint, normal);
        this.pointIndex = pointIndex;
    }


    public static ManifoldPoint fromWorld(Vector3f worldPoint, float penetration, boolean clipEdgesFromA, Vector3f normal, TransformedObject objectA, TransformedObject objectB, PointIndex pointIndex) {
        Vector3f otherPoint = normal.duplicate().mul(penetration).add(worldPoint);

        Vector3f localSpaceA = objectA.toLocalSpace(clipEdgesFromA ? worldPoint : otherPoint);
        Vector3f localSpaceB = objectB.toLocalSpace(clipEdgesFromA ? otherPoint : worldPoint);

        return new ClipManifoldPoint(localSpaceA, localSpaceB, normal, pointIndex);
    }

    @Override
    protected float replaces(ManifoldPoint other) {
        return sameIndex(other) ? 1 : super.replaces(other);
    }

    private boolean sameIndex(ManifoldPoint other) {
        return other instanceof ClipManifoldPoint && ((ClipManifoldPoint) other).pointIndex.equals(pointIndex);
    }

    @Override
    public final boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof ClipManifoldPoint)) return false;

        ClipManifoldPoint that = (ClipManifoldPoint) o;
        return Objects.equals(pointIndex, that.pointIndex);
    }

    @Override
    public int hashCode() {
        return Objects.hashCode(pointIndex);
    }
}
