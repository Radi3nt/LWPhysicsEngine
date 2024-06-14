package fr.radi3nt.physics.collision.contact.manifold;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.core.TransformedObject;

import java.util.Objects;

public class ManifoldPoint {

    public final Vector3f localSpaceAPoint;
    public final Vector3f localSpaceBPoint;

    public final Vector3f worldSpaceA = new SimpleVector3f();
    public final Vector3f worldSpaceB = new SimpleVector3f();

    public final Vector3f normalFromAToB;

    public final PointIndex index;

    public float projectedDistance;

    public float cachedContactLambda;
    public float[] cachedFrictionLambda = new float[2];

    public ManifoldPoint(PointIndex index, Vector3f localSpaceAPoint, Vector3f localSpaceBPoint, Vector3f normal) {
        this.localSpaceAPoint = localSpaceAPoint;
        this.localSpaceBPoint = localSpaceBPoint;
        this.index = index;
        this.normalFromAToB = normal;
    }

    public void refresh(TransformedObject objectA, TransformedObject objectB) {
        objectA.toWorldSpace(localSpaceAPoint, worldSpaceA);
        objectB.toWorldSpace(localSpaceBPoint, worldSpaceB);
        projectedDistance = worldSpaceB.duplicate().sub(worldSpaceA.duplicate()).dot(normalFromAToB);
    }

    public boolean orthogonalDistanceExceedThreshold(float distanceThreshold) {
        Vector3f projectedPoint = worldSpaceB.duplicate().sub(normalFromAToB.duplicate().mul(projectedDistance));
        Vector3f projectedDifference = worldSpaceA.duplicate().sub(projectedPoint);
        float dist = projectedDifference.lengthSquared();
        return dist>distanceThreshold*distanceThreshold;
    }

    public boolean exceedSignedProjectedDistanceThreshold(float threshold) {
        return projectedDistance >= threshold;
    }

    public Vector3f getDirectionFromCenterOfMassToPointA(TransformedObject a) {
        Vector3f result = localSpaceAPoint.duplicate();
        a.getRotation().transform(result);
        return result;
    }

    public Vector3f getDirectionFromCenterOfMassToPointB(TransformedObject b) {
        Vector3f result = localSpaceBPoint.duplicate();
        b.getRotation().transform(result);
        return result;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof ManifoldPoint)) return false;

        ManifoldPoint that = (ManifoldPoint) o;

        return Objects.equals(index, that.index);
    }

    @Override
    public int hashCode() {
        return index != null ? index.hashCode() : 0;
    }

    @Override
    public String toString() {
        return "ManifoldPoint{" +
                "localSpaceAPoint=" + localSpaceAPoint +
                ", localSpaceBPoint=" + localSpaceBPoint +
                ", worldSpaceA=" + worldSpaceA +
                ", worldSpaceB=" + worldSpaceB +
                ", projectedDistance=" + projectedDistance +
                '}';
    }
}
