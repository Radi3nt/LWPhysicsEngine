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
import java.util.concurrent.CopyOnWriteArrayList;

public class PersistentManifold {

    private static final int MAX_CONTACT_POINTS = 4;
    private static final float CONTACT_BREAK_NORMAL_THRESHOLD = 1e-3f;
    private static final float CONTACT_BREAK_ORTHOGONAL_THRESHOLD = 3e-2f;


    private final List<ManifoldPoint> manifoldPoints = new CopyOnWriteArrayList<>();
    private final ContactKeyPair keyPair;
    private long relevantStep;

    private GeneratedContactPair<RigidBody> currentPair;

    public PersistentManifold(GeneratedContactPair<RigidBody> keyPair) {
        this.keyPair = GeneratedContactPair.toPair(keyPair);
        currentPair = keyPair;
    }

    public void refresh(RigidBody a, RigidBody b, long currentStep) {
        currentPair = new GeneratedContactPair<>(a, keyPair.shapeA, b, keyPair.shapeB);

        for (int i = 0, manifoldPointsSize = manifoldPoints.size(); i < manifoldPointsSize; i++) {
            ManifoldPoint manifoldPoint = manifoldPoints.get(i);
            manifoldPoint.refresh(a, b);

            if (shouldRemove(manifoldPoint)) {
                manifoldPoints.remove(manifoldPoint);
                i--;
                manifoldPointsSize--;
            }
        }

        setRelevant(currentStep);
    }

    private static boolean shouldRemove(ManifoldPoint manifoldPoint) {
        return breaksSignedThreshold(manifoldPoint) || (manifoldPoint.orthogonalDistanceExceedThreshold(CONTACT_BREAK_ORTHOGONAL_THRESHOLD));
    }

    private static boolean breaksSignedThreshold(ManifoldPoint manifoldPoint) {
        return manifoldPoint.exceedSignedProjectedDistanceThreshold(CONTACT_BREAK_NORMAL_THRESHOLD);
    }

    public ContactPoint[] getContactPoints(DynamicsData a, DynamicsData b) {
        List<ContactPoint> contactPoints = new ArrayList<>();
        for (ManifoldPoint manifoldPoint : manifoldPoints) {
            contactPoints.add(manifoldPoint.toContactPoint(a, b));
        }
        return contactPoints.toArray(new ContactPoint[0]);
    }

    public void replaceManifoldPoints(Collection<ManifoldPoint> points) {
        points.removeIf(this::replacePoint);
    }

    private boolean replacePoint(ManifoldPoint point) {
        point.refresh(getObjectA(), getObjectB());

        float mostReplacing = 0;
        int mostReplacingIndex = -1;
        for (int i = 0; i < manifoldPoints.size(); i++) {
            ManifoldPoint manifoldPoint = manifoldPoints.get(i);
            if (manifoldPoint==point)
                return true;

            float replaceFactor = point.replaces(manifoldPoint);
            if (replaceFactor>mostReplacing) {
                mostReplacing = replaceFactor;
                mostReplacingIndex = i;
            }
        }
        if (mostReplacingIndex!=-1) {
            ManifoldPoint set = manifoldPoints.set(mostReplacingIndex, point);
            point.data = set.data;
            return true;
        }
        return false;
    }

    public void addManifoldPoints(Collection<ManifoldPoint> points) {
        for (ManifoldPoint point : points) {
            addManifoldPoint(point);
        }
    }

    public void addManifoldPoint(ManifoldPoint point) {
        point.refresh(getObjectA(), getObjectB());

        if (shouldRemove(point)) {
            return;
        }

        if (replacePoint(point))
            return;

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
        int deepestIndex = findDeepestIndex(point);

        float maxCurrentArea = 0;
        int maxIndex = -1;

        boolean hasDeepest = deepestIndex!=-1;

        for (int currentIndex = 0; currentIndex < manifoldPoints.size(); currentIndex++) {
            if (currentIndex==deepestIndex) {
                continue;
            }

            int firstPointIndex = hasDeepest ? deepestIndex : currentIndex;
            int secondPointIndex = (hasDeepest ? currentIndex : (currentIndex+1))%manifoldPoints.size();

            float currentArea = calc3PointsAreaSquared(point.localSpaceAPoint, manifoldPoints.get(firstPointIndex).localSpaceAPoint, manifoldPoints.get(secondPointIndex).localSpaceAPoint);
            if (currentArea > maxCurrentArea) {
                maxCurrentArea = currentArea;
                maxIndex = currentIndex;
            }
        }

        return maxIndex;
    }

    private int findDeepestIndex(ManifoldPoint point) {
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

    public void setRelevant(long currentStep) {
        this.relevantStep = currentStep;
    }
}
