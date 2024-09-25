package fr.radi3nt.physics.collision.contact.manifold.contact;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.contact.manifold.ManifoldPoint;
import fr.radi3nt.physics.core.state.DynamicsData;

public class ContactPoint {

    public static final float THRESHOLD = 2e-1f;
    public final ManifoldPoint manifoldPoint;
    public final Vector3f normal;

    public final Vector3f rA;
    public final Vector3f rB;

    public final DynamicsData a;
    public final DynamicsData b;

    private ContactType contactType;

    private float relativeVelocity;
    private Vector3f relativeVelocityVec;

    public ContactPoint(ManifoldPoint manifoldPoint, Vector3f normal, Vector3f rA, Vector3f rB, DynamicsData a, DynamicsData b) {
        this.manifoldPoint = manifoldPoint;
        this.normal = normal;
        this.rA = rA;
        this.rB = rB;
        this.a = a;
        this.b = b;
    }

    public void computeRealVelocity() {
        Vector3f velocityA = pointVelocityA();
        Vector3f velocityB = pointVelocityB();

        relativeVelocityVec = velocityB.sub(velocityA);
        relativeVelocity = normal.dot(relativeVelocityVec);

        contactType = getContactType(relativeVelocity);
    }

    public Vector3f pointVelocityB() {
        DynamicsData dynamicsData = b;

        Vector3f bodyVelocity = dynamicsData.getLinearVelocity();
        Vector3f omega = dynamicsData.getAngularVelocity();

        Vector3f angularVelocity = omega.duplicate().cross(rB);

        return angularVelocity.add(bodyVelocity);
    }

    public Vector3f pointVelocityA() {
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
