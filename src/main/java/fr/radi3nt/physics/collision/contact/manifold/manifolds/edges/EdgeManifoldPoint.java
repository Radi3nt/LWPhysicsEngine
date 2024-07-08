package fr.radi3nt.physics.collision.contact.manifold.manifolds.edges;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.contact.manifold.ManifoldPoint;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.clip.Edge;

import java.util.Objects;

public class EdgeManifoldPoint extends ManifoldPoint {

    private final Edge edgeA;
    private final Edge edgeB;

    public EdgeManifoldPoint(Vector3f localSpaceAPoint, Vector3f localSpaceBPoint, Vector3f normal, Edge edgeA, Edge edgeB) {
        super(localSpaceAPoint, localSpaceBPoint, normal);
        this.edgeA = edgeA;
        this.edgeB = edgeB;
    }

    @Override
    protected float replaces(ManifoldPoint other) {
        return sameIndex(other) ? 1 : super.replaces(other);
    }

    private boolean sameIndex(ManifoldPoint other) {
        if (!(other instanceof EdgeManifoldPoint))
            return false;
        EdgeManifoldPoint otherEdge = (EdgeManifoldPoint) other;
        return otherEdge.edgeA.equals(edgeA) && otherEdge.edgeB.equals(edgeB);
    }

    public Edge getEdgeA() {
        return edgeA;
    }

    public Edge getEdgeB() {
        return edgeB;
    }

    @Override
    public final boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof EdgeManifoldPoint)) return false;

        EdgeManifoldPoint that = (EdgeManifoldPoint) o;
        return Objects.equals(edgeA, that.edgeA) && Objects.equals(edgeB, that.edgeB);
    }

    @Override
    public int hashCode() {
        int result = Objects.hashCode(edgeA);
        result = 31 * result + Objects.hashCode(edgeB);
        return result;
    }
}
