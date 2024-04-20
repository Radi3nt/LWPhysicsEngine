package fr.radi3nt.physics.collision.contact.manifold;

import java.util.Objects;

public class PointIndex {

    public final ClipIndex clipIndex;
    public final boolean aObject;

    public PointIndex(ClipIndex clipIndex, boolean aObject) {
        this.clipIndex = clipIndex;
        this.aObject = aObject;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof PointIndex)) return false;

        PointIndex index = (PointIndex) o;

        if (aObject != index.aObject) return false;
        return Objects.equals(clipIndex, index.clipIndex);
    }

    @Override
    public int hashCode() {
        int result = clipIndex != null ? clipIndex.hashCode() : 0;
        result = 31 * result + (aObject ? 1 : 0);
        return result;
    }
}
