package fr.radi3nt.physics.collision.contact.manifold;

public class ClipIndex {

    public final int clippedEdge;
    public final int clippedPlane;

    public ClipIndex(int clippedEdge, int clippedPlane) {
        this.clippedEdge = clippedEdge;
        this.clippedPlane = clippedPlane;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof ClipIndex)) return false;

        ClipIndex index = (ClipIndex) o;

        if (clippedEdge != index.clippedEdge) return false;
        return clippedPlane == index.clippedPlane;
    }

    @Override
    public int hashCode() {
        int result = clippedEdge;
        result = 31 * result + clippedPlane;
        return result;
    }
}
