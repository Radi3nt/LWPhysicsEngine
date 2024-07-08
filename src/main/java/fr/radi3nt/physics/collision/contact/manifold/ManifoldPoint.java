package fr.radi3nt.physics.collision.contact.manifold;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.contact.manifold.contact.ContactPoint;
import fr.radi3nt.physics.core.TransformedObject;
import fr.radi3nt.physics.core.state.DynamicsData;

public class ManifoldPoint {

    private static final float MERGE_THRESHOLD = 1e-2f;

    public final Vector3f localSpaceAPoint;
    public final Vector3f localSpaceBPoint;

    public final Vector3f worldSpaceA = new SimpleVector3f();
    public final Vector3f rA = new SimpleVector3f();
    public final Vector3f worldSpaceB = new SimpleVector3f();
    public final Vector3f rB = new SimpleVector3f();

    public final Vector3f normalFromAToB;

    public float projectedDistance;

    public ManifoldPointData data = new ManifoldPointData();

    public ManifoldPoint(Vector3f localSpaceAPoint, Vector3f localSpaceBPoint, Vector3f normal) {
        this.localSpaceAPoint = localSpaceAPoint;
        this.localSpaceBPoint = localSpaceBPoint;
        this.normalFromAToB = normal;
    }

    public static ManifoldPoint fromWorld(Vector3f worldPoint, float penetration, boolean clipEdgesFromA, Vector3f normal, TransformedObject objectA, TransformedObject objectB) {
        Vector3f otherPoint = normal.duplicate().mul(clipEdgesFromA ? -penetration : penetration).add(worldPoint);

        Vector3f localSpaceA = objectA.toLocalSpace(clipEdgesFromA ? worldPoint : otherPoint);
        Vector3f localSpaceB = objectB.toLocalSpace(clipEdgesFromA ? otherPoint : worldPoint);

        return new ManifoldPoint(localSpaceA, localSpaceB, normal);
    }

    public ContactPoint toContactPoint(DynamicsData a, DynamicsData b) {
        return new ContactPoint(this, normalFromAToB.duplicate(), rA, rB, a, b);
    }

    protected float replaces(ManifoldPoint other) {
        float dist = localSpaceAPoint.duplicate().sub(other.localSpaceAPoint).lengthSquared();
        return dist<MERGE_THRESHOLD*MERGE_THRESHOLD ? (MERGE_THRESHOLD*MERGE_THRESHOLD-dist)/(MERGE_THRESHOLD*MERGE_THRESHOLD) : -1;
    }

    public void refresh(TransformedObject objectA, TransformedObject objectB) {
        objectA.toWorldSpace(localSpaceAPoint, worldSpaceA);
        objectB.toWorldSpace(localSpaceBPoint, worldSpaceB);
        rA.copy(getDirectionFromCenterOfMassToPointA(objectA));
        rB.copy(getDirectionFromCenterOfMassToPointB(objectB));
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

    private Vector3f getDirectionFromCenterOfMassToPointA(TransformedObject a) {
        Vector3f result = localSpaceAPoint.duplicate();
        a.getRotation().transform(result);
        return result;
    }

    private Vector3f getDirectionFromCenterOfMassToPointB(TransformedObject b) {
        Vector3f result = localSpaceBPoint.duplicate();
        b.getRotation().transform(result);
        return result;
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
