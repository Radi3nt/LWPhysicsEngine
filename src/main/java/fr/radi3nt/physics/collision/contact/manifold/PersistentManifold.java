package fr.radi3nt.physics.collision.contact.manifold;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.contact.ContactPair;
import fr.radi3nt.physics.collision.contact.manifold.contact.ContactPoint;
import fr.radi3nt.physics.core.TransformedObject;
import fr.radi3nt.physics.core.state.DynamicsData;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.abs;

public class PersistentManifold {

    private TransformedObject objectA;
    private TransformedObject objectB;
    private final List<ManifoldPoint> manifoldPoints = new ArrayList<>();
    private long relevantStep;

    private final Vector3f contactNormal = new SimpleVector3f();
    private float distanceThreshold;

    public PersistentManifold(ContactPair contactPair) {
        this.objectA = contactPair.objectA;
        this.objectB = contactPair.objectB;
    }

    public void setPair(TransformedObject a, TransformedObject b) {
        objectA = a;
        objectB = b;
    }

    public void refresh() {
        for (ManifoldPoint manifoldPoint : manifoldPoints) {
            manifoldPoint.refresh(objectA, objectB, contactNormal);
        }
    }

    public void removeInvalidPoints() {
        for (int i = 0; i < manifoldPoints.size(); i++) {
            ManifoldPoint manifoldPoint = manifoldPoints.get(i);
            boolean signedDist = manifoldPoint.exceedSignedProjectedDistanceThreshold(2);
            if (signedDist || manifoldPoint.orthogonalDistanceExceedThreshold(2)) {
                manifoldPoints.remove(manifoldPoint);
                i--;
            }
        }
    }

    public void setDistanceThreshold(float distanceThreshold) {
        this.distanceThreshold = distanceThreshold;
    }

    public ContactPoint[] getContactPoints(DynamicsData a, DynamicsData b) {
        List<ContactPoint> contactPoints = new ArrayList<>();
        for (ManifoldPoint manifoldPoint : manifoldPoints) {
            if (manifoldPoint.enabled)
                contactPoints.add(new ContactPoint(manifoldPoint, contactNormal, manifoldPoint.getDirectionFromCenterOfMassToPointA(objectA), manifoldPoint.getDirectionFromCenterOfMassToPointB(objectB), a, b));
        }
        return contactPoints.toArray(new ContactPoint[0]);
    }

    public Vector3f getContactNormal() {
        return contactNormal;
    }

    public void setNormal(Vector3f contactNormal) {
        this.contactNormal.copy(contactNormal);
    }

    public TransformedObject getObjectA() {
        return objectA;
    }

    public TransformedObject getObjectB() {
        return objectB;
    }

    public List<ManifoldPoint> getManifoldPoints() {
        return manifoldPoints;
    }

    public boolean isEmpty() {
        for (ManifoldPoint manifoldPoint : manifoldPoints) {
            if (manifoldPoint.enabled)
                return false;
        }
        return true;
    }

    public boolean isRelevant(long step, int stepThreshold) {
        return abs(this.relevantStep-step)<=stepThreshold;
    }

    public PersistentManifold setRelevant(long currentStep) {
        this.relevantStep = currentStep;
        return this;
    }
}
