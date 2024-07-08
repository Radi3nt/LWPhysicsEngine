package fr.radi3nt.physics.collision.contact.manifold.manifolds.clip;

public class ClipIndex {

    public final int clippedEdge;
    public final boolean secondVertex;
    public final int clippedPlane;

    public ClipIndex(int clippedEdge, boolean secondVertex, int clippedPlane) {
        this.clippedEdge = clippedEdge;
        this.secondVertex = secondVertex;
        this.clippedPlane = clippedPlane;
    }

    @Override
    public final boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof ClipIndex)) return false;

        ClipIndex clipIndex = (ClipIndex) o;
        return clippedEdge == clipIndex.clippedEdge && secondVertex == clipIndex.secondVertex && clippedPlane == clipIndex.clippedPlane;
    }

    @Override
    public int hashCode() {
        int result = clippedEdge;
        result = 31 * result + Boolean.hashCode(secondVertex);
        result = 31 * result + clippedPlane;
        return result;
    }
}
