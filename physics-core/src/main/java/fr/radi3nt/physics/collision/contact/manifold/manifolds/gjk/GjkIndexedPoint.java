package fr.radi3nt.physics.collision.contact.manifold.manifolds.gjk;

public class GjkIndexedPoint implements Comparable<GjkIndexedPoint> {

    private final int aVertexId;
    private final int bVertexId;

    public GjkIndexedPoint(int aVertexId, int bVertexId) {
        this.aVertexId = aVertexId;
        this.bVertexId = bVertexId;
    }

    @Override
    public final boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof GjkIndexedPoint)) return false;

        GjkIndexedPoint that = (GjkIndexedPoint) o;
        return aVertexId == that.aVertexId && bVertexId == that.bVertexId;
    }

    @Override
    public int hashCode() {
        int result = aVertexId;
        result = 31 * result + bVertexId;
        return result;
    }

    @Override
    public int compareTo(GjkIndexedPoint o) {
        int compare = Integer.compare(o.aVertexId, aVertexId);
        if (compare==0)
            return Integer.compare(o.bVertexId, bVertexId);
        return compare;
    }
}
