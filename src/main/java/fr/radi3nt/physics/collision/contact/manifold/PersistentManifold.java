package fr.radi3nt.physics.collision.contact.manifold;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.contact.ContactKeyPair;
import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.contact.manifold.contact.ContactPoint;
import fr.radi3nt.physics.core.state.DynamicsData;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class PersistentManifold {

    private static final int MAX_CONTACT_POINTS = 4;
    private static final float CONTACT_BREAK_THRESHOLD = 1e-2f;

    private final List<ManifoldPoint> manifoldPoints = new ArrayList<>();
    private final ContactKeyPair keyPair;
    private long relevantStep;

    private GeneratedContactPair currentPair;

    public PersistentManifold(ContactKeyPair keyPair) {
        this.keyPair = keyPair;
    }

    public void refresh(RigidBody a, RigidBody b) {
        currentPair = new GeneratedContactPair(a, keyPair.shapeA, b, keyPair.shapeB);

        for (int i = 0, manifoldPointsSize = manifoldPoints.size(); i < manifoldPointsSize; i++) {
            ManifoldPoint manifoldPoint = manifoldPoints.get(i);
            manifoldPoint.refresh(a, b);

            if (shouldRemove(manifoldPoint)) {
                manifoldPoints.remove(manifoldPoint);
                i--;
                manifoldPointsSize--;
            }
        }
    }

    private static boolean shouldRemove(ManifoldPoint manifoldPoint) {
        return breaksSignedThreshold(manifoldPoint) || (manifoldPoint.orthogonalDistanceExceedThreshold(CONTACT_BREAK_THRESHOLD));
    }

    private static boolean breaksSignedThreshold(ManifoldPoint manifoldPoint) {
        return manifoldPoint.exceedSignedProjectedDistanceThreshold(CONTACT_BREAK_THRESHOLD);
    }

    public ContactPoint[] getContactPoints(DynamicsData a, DynamicsData b) {
        List<ContactPoint> contactPoints = new ArrayList<>();
        for (ManifoldPoint manifoldPoint : manifoldPoints) {
            contactPoints.add(new ContactPoint(manifoldPoint, manifoldPoint.normalFromAToB.duplicate(), manifoldPoint.getDirectionFromCenterOfMassToPointA(a), manifoldPoint.getDirectionFromCenterOfMassToPointB(b), a, b));
        }
        return contactPoints.toArray(new ContactPoint[0]);
    }


    public void addManifoldPoints(Collection<ManifoldPoint> points) {
        for (ManifoldPoint point : points) {
            addManifoldPoint(point);
        }
    }

    public void addManifoldPoint(ManifoldPoint point) {
        point.refresh(getObjectA(), getObjectB());

        if (breaksSignedThreshold(point)) {
            return;
        }

        if (manifoldPoints.size()<MAX_CONTACT_POINTS) {
            manifoldPoints.add(point);
            return;
        }

        int replacingIndex = findReplacingIndex(point);
        if (replacingIndex==-1)
            return;
        manifoldPoints.set(replacingIndex, point);
    }

    private int findReplacingIndex(ManifoldPoint point) {
        int shallowestIndex = findShallowestIndex(point);

        float maxCurrentArea = 0;
        int maxIndex = -1;

        for (int currentIndex = 0; currentIndex < manifoldPoints.size(); currentIndex++) {
            if (currentIndex==shallowestIndex) {
                continue;
            }

            float currentArea = calc3PointsAreaSquared(point.localSpaceAPoint, manifoldPoints.get(currentIndex%(manifoldPoints.size()-1)).localSpaceAPoint, manifoldPoints.get((currentIndex+1)%(manifoldPoints.size()-1)).localSpaceAPoint);
            if (currentArea > maxCurrentArea) {
                maxCurrentArea = currentArea;
                maxIndex = currentIndex;
            }
        }

        return maxIndex;
    }

    private int findShallowestIndex(ManifoldPoint point) {
        float maxPenetration = point.projectedDistance;
        int replacingIndex = -1;
        for (int i = 0, manifoldPointsSize = manifoldPoints.size(); i < manifoldPointsSize; i++) {
            ManifoldPoint manifoldPoint = manifoldPoints.get(i);
            if (manifoldPoint.projectedDistance < maxPenetration) {
                replacingIndex = i;
                maxPenetration = manifoldPoint.projectedDistance;
            }
        }
        return replacingIndex;
    }


    private float calc3PointsAreaSquared(Vector3f point1, Vector3f point2, Vector3f point3) {
        Vector3f a0 = point1.duplicate().sub(point2);
        Vector3f b0 = point3.duplicate().sub(point2);
        Vector3f cross = a0.cross(b0);
        return cross.lengthSquared();
    }

    public ContactPoint[] getCurrentContactPoints() {
        return getContactPoints(currentPair.objectA.getDynamicsData(), currentPair.objectB.getDynamicsData());
    }

    public RigidBody getObjectA() {
        return currentPair.objectA;
    }


    public RigidBody getObjectB() {
        return currentPair.objectB;
    }

    public List<ManifoldPoint> getManifoldPoints() {
        return manifoldPoints;
    }

    public void clear() {
        manifoldPoints.clear();
    }

    public boolean isEmpty() {
        return manifoldPoints.isEmpty();
    }

    public boolean isRelevant(long step, int stepThreshold) {
        return step-this.relevantStep<=stepThreshold;
    }

    public PersistentManifold setRelevant(long currentStep) {
        this.relevantStep = currentStep;
        return this;
    }
}
