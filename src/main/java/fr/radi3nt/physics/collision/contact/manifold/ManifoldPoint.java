package fr.radi3nt.physics.collision.contact.manifold;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.CollisionShapeGroup;
import fr.radi3nt.physics.core.TransformedObject;

import java.util.Objects;

import static java.lang.Math.abs;

public class ManifoldPoint {

    public final Vector3f localSpaceAPoint;
    public final Vector3f localSpaceBPoint;

    public final Vector3f worldSpaceA = new SimpleVector3f();
    public final Vector3f worldSpaceB = new SimpleVector3f();

    public final PointIndex index;

    public float projectedDistance;

    public float cachedContactLambda;
    public float[] cachedFrictionLambda = new float[2];

    public boolean enabled = false;

    public ManifoldPoint(PointIndex index, Vector3f localSpaceAPoint, Vector3f localSpaceBPoint) {
        this.localSpaceAPoint = localSpaceAPoint;
        this.localSpaceBPoint = localSpaceBPoint;
        this.index = index;
    }

    public void refresh(TransformedObject objectA, TransformedObject objectB, Vector3f projectingNormal) {
        objectA.toWorldSpace(localSpaceAPoint, worldSpaceA);
        objectB.toWorldSpace(localSpaceBPoint, worldSpaceB);
        projectedDistance = worldSpaceA.duplicate().sub(worldSpaceB).dot(projectingNormal);
    }

    public boolean orthogonalDistanceExceedThreshold(float distanceThreshold) {
        Vector3f projectedPoint = worldSpaceA.duplicate().sub(worldSpaceB.duplicate().mul(projectedDistance));
        Vector3f projectedDifference = worldSpaceB.duplicate().sub(projectedPoint);
        float dist = projectedDifference.lengthSquared();
        return dist>=distanceThreshold*distanceThreshold;
    }

    public boolean exceedSignedProjectedDistanceThreshold(float threshold) {
        return projectedDistance >= threshold;
    }

    public boolean exceedDistanceThreshold(float threshold) {
        double dist = worldSpaceA.duplicate().sub(worldSpaceB).lengthSquared();
        return dist >= threshold*threshold;
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
