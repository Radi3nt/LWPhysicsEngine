package fr.radi3nt.physics.collision.contact.manifold.contact;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.contact.manifold.ManifoldPoint;
import fr.radi3nt.physics.core.state.DynamicsData;

public class ContactPoint {

    public static final float THRESHOLD = 1e-3f;
    public final ManifoldPoint manifoldPoint;
    public final Vector3f normal;

    public final Vector3f rA;
    public final Vector3f rB;

    public final DynamicsData a;
    public final DynamicsData b;

    public final float penetration;

    private ContactType contactType;

    private float relativeVelocity;
    private Vector3f relativeVelocityVec;

    public ContactPoint(ManifoldPoint manifoldPoint, Vector3f normal, Vector3f rA, Vector3f rB, DynamicsData a, DynamicsData b, float penetration) {
        this.manifoldPoint = manifoldPoint;
        this.normal = normal;
        this.rA = rA;
        this.rB = rB;
        this.a = a;
        this.b = b;
        this.penetration = penetration;
    }

    public void computeRealVelocity() {
        computeRealVelocity(false, false);
    }

    public void computeRealVelocity(boolean aSleep, boolean bSleep) {
        Vector3f velocityA = pointVelocityA(aSleep);
        Vector3f velocityB = pointVelocityB(bSleep);

        relativeVelocityVec = velocityB.sub(velocityA);
        relativeVelocity = normal.dot(relativeVelocityVec);

        contactType = getContactType(relativeVelocity);
    }

    public Vector3f pointVelocityB() {
        return pointVelocityB(false);
    }

    public Vector3f pointVelocityB(boolean bSleep) {
        if (bSleep)
            return new SimpleVector3f();
        DynamicsData dynamicsData = b;

        Vector3f bodyVelocity = dynamicsData.getLinearVelocity();
        Vector3f omega = dynamicsData.getAngularVelocity();

        Vector3f angularVelocity = omega.duplicate().cross(rB);

        return angularVelocity.add(bodyVelocity);
    }

    public Vector3f pointVelocityA() {
        return pointVelocityA(false);
    }

    public Vector3f pointVelocityA(boolean aSleep) {
        if (aSleep)
            return new SimpleVector3f();
        DynamicsData dynamicsData = a;

        Vector3f bodyVelocity = dynamicsData.getLinearVelocity();
        Vector3f omega = dynamicsData.getAngularVelocity();

        Vector3f angularVelocity = omega.duplicate().cross(rA);

        return angularVelocity.add(bodyVelocity);
    }

    public boolean isCollidingContact() {
        return contactType==ContactType.COLLIDING;
    }

    public boolean isRestingContact() {
        return contactType==ContactType.RESTING;
    }

    public boolean isSeparatingContact() {
        return contactType==ContactType.SEPARATING;
    }


    public ContactType getContactType() {
        return contactType;
    }

    private ContactType getContactType(float dot) {
        return ContactType.fromDot(dot, THRESHOLD);
    }

    public float getRelativeVelocityAlongNormal() {
        return relativeVelocity;
    }

    public Vector3f getRelativeVelocityVec() {
        return relativeVelocityVec;
    }

}
