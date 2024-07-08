package fr.radi3nt.physics.collision.contact.manifold.manifolds.gjk;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.contact.manifold.ManifoldPoint;

import java.util.Arrays;

public class GJKManifoldPoint extends ManifoldPoint {

    private final GjkIndexedPoint[] indexedPoints;

    public GJKManifoldPoint(Vector3f localSpaceAPoint, Vector3f localSpaceBPoint, Vector3f normal, GjkIndexedPoint[] indexedPoints) {
        super(localSpaceAPoint, localSpaceBPoint, normal);
        this.indexedPoints = indexedPoints;
    }

    @Override
    protected float replaces(ManifoldPoint other) {
        return other instanceof GJKManifoldPoint && Arrays.equals(((GJKManifoldPoint) other).indexedPoints, indexedPoints) ? 1 : super.replaces(other);
    }

    @Override
    public final boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof GJKManifoldPoint)) return false;

        GJKManifoldPoint that = (GJKManifoldPoint) o;
        return Arrays.equals(indexedPoints, that.indexedPoints);
    }

    @Override
    public int hashCode() {
        return Arrays.hashCode(indexedPoints);
    }
}
