package fr.radi3nt.physics.collision.response.sequential;

import fr.radi3nt.maths.components.advanced.matrix.Matrix3x3;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.collision.contact.manifold.contact.ContactPoint;
import fr.radi3nt.physics.collision.contact.manifold.contact.ContactType;
import fr.radi3nt.physics.collision.response.CollisionContactSolver;
import fr.radi3nt.physics.core.state.DynamicsData;
import fr.radi3nt.physics.sleeping.SleepingData;

import java.util.Collection;

import static java.lang.Math.abs;

public class SequentialImpulseCollisionContactSolver implements CollisionContactSolver {

    private static final float WAKE_THRESHOLD = 1e-3f;
    private final int iterationCount;

    public SequentialImpulseCollisionContactSolver(int iterationCount) {
        this.iterationCount = iterationCount;
    }

    @Override
    public void solve(Collection<PersistentManifold> manifolds, float dt) {
        for (PersistentManifold manifold : manifolds) {
            computeManifold(manifold, manifold.getObjectA().getDynamicsData(), manifold.getObjectB().getDynamicsData(), manifold.getObjectA().getSleepingData(), manifold.getObjectB().getSleepingData());
        }
    }

    private void computeManifold(PersistentManifold manifold, DynamicsData a, DynamicsData b, SleepingData sleepA, SleepingData sleepB) {
        ContactPoint[] contactPoints = manifold.getContactPoints(a, b);
        float[] staticValues = new float[contactPoints.length];
        float[] initialRelativeVel = new float[contactPoints.length];
        Vector3f[] initialRelativeVec = new Vector3f[contactPoints.length];

        for (int i = 0; i < contactPoints.length; i++) {
            ContactPoint contactPoint = contactPoints[i];
            Vector3f normal = contactPoint.normal;

            Vector3f ra = contactPoint.rA;
            Vector3f rb = contactPoint.rB;

            Matrix3x3 iInvA = a.getIInv();
            Matrix3x3 iInvB = b.getIInv();

            Vector3f inertiaA = ra.duplicate().cross(normal);
            Vector3f inertiaB = rb.duplicate().cross(normal);

            iInvA.transform(inertiaA);
            iInvB.transform(inertiaB);

            inertiaA.cross(ra.duplicate());
            inertiaB.cross(rb.duplicate());

            float totalMass = a.getBodyProperties().inverseMass + b.getBodyProperties().inverseMass;
            float angularEffect = inertiaA.add(inertiaB).dot(normal);

            staticValues[i] = (totalMass + angularEffect);

            contactPoint.computeRealVelocity();
            initialRelativeVel[i] = contactPoint.getRelativeVelocityAlongNormal();
            initialRelativeVec[i] = contactPoint.getRelativeVelocityVec().duplicate();
        }

        removeOverlap(a, b, sleepA, sleepB, contactPoints, initialRelativeVel, staticValues);

        for (ContactPoint contactPoint : contactPoints) {
            contactPoint.computeRealVelocity();
        }

        applyBounce(a, b, contactPoints, initialRelativeVel, initialRelativeVec, staticValues);

    }

    private static void applyBounce(DynamicsData a, DynamicsData b, ContactPoint[] contactPoints, float[] initialRelativeVel, Vector3f[] initialRelativeVec, float[] staticValues) {
        for (int i = 0; i < contactPoints.length; i++) {
            ContactPoint contactPoint = contactPoints[i];
            float relativeVel = initialRelativeVel[i];
            if (relativeVel >= 0)
                continue;

            Vector3f relativeVelVec = initialRelativeVec[i];

            float epsilon = a.getBodyProperties().bouncingFactor * b.getBodyProperties().bouncingFactor;
            if (epsilon <= 0)
                continue;


            Vector3f normal = contactPoint.normal;

            Vector3f ra = contactPoint.rA;
            Vector3f rb = contactPoint.rB;

            float numerator = -((epsilon) * relativeVel);

            Vector3f frictionVec = getFrictionVec(a, b, normal, relativeVelVec, relativeVel);
            Vector3f dir = normal.duplicate().add(frictionVec);

            float j = numerator / staticValues[i];
            Vector3f fullImpulse = dir.duplicate().mul(j);

            b.addLinearImpulse(fullImpulse);
            b.addAngularImpulse(rb.duplicate().cross(fullImpulse.duplicate()));

            a.addLinearImpulse(fullImpulse.duplicate().negate());
            a.addAngularImpulse(ra.duplicate().cross(fullImpulse.duplicate().negate()));
        }
    }

    private static Vector3f getFrictionVec(DynamicsData a, DynamicsData b, Vector3f normal, Vector3f relativeVelVec, float impulseForce) {
        float uk = a.getBodyProperties().kineticFrictionFactor * b.getBodyProperties().kineticFrictionFactor;
        float us = a.getBodyProperties().staticFrictionFactor * b.getBodyProperties().staticFrictionFactor;
        float frictionFactor = uk;

        Vector3f t = normal.duplicate().cross(normal.duplicate().cross(relativeVelVec.duplicate()));
        float currentNormalForce = abs(relativeVelVec.dot(normal));
        float currentTangentForce = abs(relativeVelVec.dot(t));
        float forceRequiredToSlide = us * abs(impulseForce);
        if (currentTangentForce < forceRequiredToSlide)
            frictionFactor = us * currentTangentForce;

        if (currentNormalForce > 0.99f)
            frictionFactor = 0;

        return t.mul(frictionFactor);
    }

    private void removeOverlap(DynamicsData a, DynamicsData b, SleepingData sleepA, SleepingData sleepB, ContactPoint[] contactPoints, float[] initialRelativeVel, float[] staticValues) {
        float[] accumulatedImpulses = new float[initialRelativeVel.length];

        resolveOverlap(a, b, contactPoints, staticValues, accumulatedImpulses);

        for (int i = 0; i < accumulatedImpulses.length; i++) {
            float accumulatedImpulse = accumulatedImpulses[i];
            float relativeVel = initialRelativeVel[i];
            if (accumulatedImpulse > WAKE_THRESHOLD && ContactType.fromDot(relativeVel, ContactPoint.THRESHOLD)==ContactType.COLLIDING) {
                sleepA.wakeUpIfNeeded(a);
                if (!sleepA.isSleeping())
                    sleepB.wakeUpIfNeeded();
                sleepB.wakeUpIfNeeded(b);
                if (!sleepB.isSleeping())
                    sleepA.wakeUpIfNeeded();
            }
        }
    }

    private void resolveOverlap(DynamicsData a, DynamicsData b, ContactPoint[] contactPoints, float[] staticValues, float[] accumulatedImpulses) {
        for (int currentIteration = 0; currentIteration < iterationCount; currentIteration++) {
            boolean atLeastOneContactCollided = false;

            for (ContactPoint contactPoint : contactPoints) {
                contactPoint.computeRealVelocity();

                if (contactPoint.getContactType()== ContactType.COLLIDING)
                    atLeastOneContactCollided = true;
            }

            if (!atLeastOneContactCollided)
                return;

            for (int i = 0; i < contactPoints.length; i++) {
                ContactPoint contactPoint = contactPoints[i];
                float impulseForce = contactPoint.getRelativeVelocityAlongNormal();

                Vector3f normal = contactPoint.normal;

                Vector3f ra = contactPoint.rA;
                Vector3f rb = contactPoint.rB;

                float numerator = -(impulseForce);

                Vector3f dir = normal.duplicate();
                dir.add(getFrictionVec(a, b, normal, contactPoint.getRelativeVelocityVec(), impulseForce));

                float j = numerator / staticValues[i];
                float force = j / (iterationCount);
                float oldAccumulatedImpulse = accumulatedImpulses[i];
                accumulatedImpulses[i] = Math.max(force+oldAccumulatedImpulse, 0);

                float difference = accumulatedImpulses[i]-oldAccumulatedImpulse;
                if (difference==0)
                    continue;

                Vector3f fullImpulse = dir.duplicate().mul(difference);

                b.addLinearImpulse(fullImpulse);
                b.addAngularImpulse(rb.duplicate().cross(fullImpulse.duplicate()));

                a.addLinearImpulse(fullImpulse.duplicate().negate());
                a.addAngularImpulse(ra.duplicate().cross(fullImpulse.duplicate().negate()));
            }
        }
    }

}
