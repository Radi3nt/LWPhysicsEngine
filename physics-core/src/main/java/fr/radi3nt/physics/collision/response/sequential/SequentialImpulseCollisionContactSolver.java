package fr.radi3nt.physics.collision.response.sequential;

import fr.radi3nt.maths.Maths;
import fr.radi3nt.maths.components.advanced.matrix.Matrix3x3;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.collision.contact.manifold.contact.ContactPoint;
import fr.radi3nt.physics.collision.response.CollisionContactSolver;
import fr.radi3nt.physics.core.state.DynamicsData;
import fr.radi3nt.physics.main.Simulation;
import fr.radi3nt.physics.sleeping.SleepingData;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import static java.lang.Math.abs;

public class SequentialImpulseCollisionContactSolver implements CollisionContactSolver {

    private static final float WAKE_THRESHOLD = 1e-1f;
    private static final float SLOP_PENETRATION = 1e-3f;
    private static final float BIAS_FACTOR = 0.2f;
    private static final float BOUNCE_IMPULSE_THRESHOLD = 0.3f;
    private static final float DELTA_THRESHOLD = 3e-4f;
    private static final double INV_SQRT2 = 0.7071067811865475244008443621048490d;

    private final Simulation simulation;
    private final int iterationCount;

    public SequentialImpulseCollisionContactSolver(Simulation simulation, int iterationCount) {
        this.simulation = simulation;
        this.iterationCount = iterationCount;
    }

    @Override
    public void solve(Collection<PersistentManifold> manifolds, float dt) {
        Collection<Collection<SequentialPoint>> manifoldsSequences = new ArrayList<>();
        for (PersistentManifold manifold : manifolds) {
            computePoints(manifoldsSequences, manifold, dt);
        }

        for (Collection<SequentialPoint> points : manifoldsSequences) {
            for (SequentialPoint sequentialPoint : points) {
                sequentialPoint.preStep();
            }
        }


        for (int i = 0; i < iterationCount; i++) {
            float mostDelta = 0;
            for (Collection<SequentialPoint> points : manifoldsSequences) {
                for (SequentialPoint sequentialPoint : points) {
                    mostDelta = Math.max(sequentialPoint.correct(), mostDelta);
                }
            }
            if (mostDelta<=DELTA_THRESHOLD) {
                break;
            }
        }
        for (Collection<SequentialPoint> points : manifoldsSequences) {
            for (int i = 0; i < 10; i++) {
                for (SequentialPoint sequentialPoint : points) {
                    sequentialPoint.applyBounce();
                }
            }
        }
        for (Collection<SequentialPoint> points : manifoldsSequences) {
            for (SequentialPoint sequentialPoint : points) {
                sequentialPoint.zeroIfSleeping();
                sequentialPoint.wakeIfNeeded();
            }
        }
    }

    private void computePoints(Collection<Collection<SequentialPoint>> sequentialPoints, PersistentManifold manifold, float dt) {
        DynamicsData a = manifold.getObjectA().getDynamicsData();
        DynamicsData b = manifold.getObjectB().getDynamicsData();
        ContactPoint[] contactPoints = manifold.getContactPoints(a, b);
        List<SequentialPoint> points = new ArrayList<>();
        for (int i = 0, contactPointsLength = contactPoints.length; i < contactPointsLength; i++) {
            ContactPoint contactPoint = contactPoints[(int) ((i+simulation.getStep())%contactPointsLength)];
            Vector3f normal = contactPoint.normal;

            if (normal.lengthSquared()==0)
                continue;

            boolean aSleeping = !manifold.getObjectA().getSleepingData().isAwoken();
            boolean bSleeping = !manifold.getObjectB().getSleepingData().isAwoken();

            contactPoint.computeRealVelocity(aSleeping, bSleeping);

            if (!contactPoint.isCollidingContact()) {
                contactPoint.manifoldPoint.data.cachedAccumulatedImpulse = 0;
                contactPoint.manifoldPoint.data.cachedAccumulatedFrictionLambda[0] = 0;
                contactPoint.manifoldPoint.data.cachedAccumulatedFrictionLambda[1] = 0;
            }

            Vector3f ra = contactPoint.rA;
            Vector3f rb = contactPoint.rB;

            Matrix3x3 iInvA = a.getIInv();
            Matrix3x3 iInvB = b.getIInv();

            if (aSleeping)
                iInvA.zero();
            if (bSleeping)
                iInvB.zero();

            Vector3f inertiaAN = ra.duplicate().cross(normal);
            Vector3f inertiaBN = rb.duplicate().cross(normal);


            iInvA.transform(inertiaAN);
            iInvB.transform(inertiaBN);

            inertiaAN.cross(ra.duplicate());
            inertiaBN.cross(rb.duplicate());



            float localAMass = aSleeping ? 0 : a.getBodyProperties().inverseMass;
            float totalMass = localAMass + (bSleeping ? 0 : b.getBodyProperties().inverseMass);
            float kn = (totalMass + inertiaAN.add(inertiaBN).dot(normal));

            if (kn==0)
                continue;

            points.add(new SequentialPoint(totalMass, kn, contactPoint.getRelativeVelocityAlongNormal(), contactPoint, a, b, manifold.getObjectA().getSleepingData(), manifold.getObjectB().getSleepingData(), dt));
        }
        sequentialPoints.add(points);
    }

    public static void computeTangents(Vector3f tangentA, Vector3f tangentB, Vector3f normal, Vector3f relativeVel) {
        tangentA.copy(getTangent(normal, relativeVel).normalizeSafely());
        if (tangentA.lengthSquared() == 0) {
            tangentialPlane(normal, tangentA, tangentB);
        } else {
            tangentB.copy(normal.duplicate().cross(tangentA));
        }
    }

    private static void tangentialPlane(Vector3f normal, Vector3f tangentA, Vector3f tangentB) {
        if (abs(normal.getZ())> INV_SQRT2) {
            float a = normal.getY()*normal.getY() + normal.getZ()*normal.getZ();
            float k = (float) (1f/Math.sqrt(a));
            tangentA.set(0, -normal.getZ()*k, normal.getY()*k);
            tangentB.set(a*k, -normal.getX()*tangentA.getZ(), normal.getX()*tangentA.getY());
        } else {
            float a = normal.getX()*normal.getX() + normal.getY()*normal.getY();
            float k = (float) (1f/Math.sqrt(a));
            tangentA.set(-normal.getY()*k, normal.getX()*k, 0);
            tangentB.set(-normal.getZ()*tangentA.getY(), normal.getZ()*tangentA.getX(), a*k);
        }

    }

    private static Vector3f getTangent(Vector3f normal, Vector3f relativeVelVec) {
        return relativeVelVec.duplicate().sub(normal.duplicate().mul(relativeVelVec.dot(normal)));
    }

    public static class SequentialPoint {

        private static final boolean WAKE_UP_MANUALLY = true;
        private static final boolean BIAS = true;
        private static final float MIN_BOUNCE_VELOCITY = 1f;

        private final Vector3f tangentA = new SimpleVector3f();
        private final Vector3f tangentB = new SimpleVector3f();
        private final float totalMass;
        private final float kN;
        private float kTa;
        private float kTb;
        private final float initialRelativeVelocity;
        private final ContactPoint contactPoint;
        private final DynamicsData a;
        private final DynamicsData b;

        private final SleepingData sleepA;
        private final SleepingData sleepB;

        private float accumulatedImpulse;
        private float totalImpulse;
        private float accumulatedFrictionA;
        private float accumulatedFrictionB;

        private float velBias;


        public SequentialPoint(float totalMass, float kN, float initialRelativeVelocity, ContactPoint contactPoint, DynamicsData a, DynamicsData b, SleepingData sleepA, SleepingData sleepB, float delta) {
            this.totalMass = totalMass;
            this.kN = kN;
            this.initialRelativeVelocity = initialRelativeVelocity;
            this.contactPoint = contactPoint;
            this.a = a;
            this.b = b;
            this.sleepA = sleepA;
            this.sleepB = sleepB;
            this.velBias = (BIAS_FACTOR/delta) * Math.max(-contactPoint.penetration - SLOP_PENETRATION, 0);

            this.tangentA.copy(contactPoint.manifoldPoint.data.oldTa);
            this.tangentB.copy(contactPoint.manifoldPoint.data.oldTb);
            this.accumulatedImpulse = contactPoint.manifoldPoint.data.cachedAccumulatedImpulse;
            this.accumulatedFrictionA = contactPoint.manifoldPoint.data.cachedAccumulatedFrictionLambda[0];
            this.accumulatedFrictionB = contactPoint.manifoldPoint.data.cachedAccumulatedFrictionLambda[1];
        }

        private void projectsTangentImpulses(Vector3f newTa, Vector3f newTb) {
            Vector3f friction = tangentA.duplicate().mul(accumulatedFrictionA).add(tangentB.duplicate().mul(accumulatedFrictionB));
            this.accumulatedFrictionA = newTa.dot(friction);
            this.accumulatedFrictionB = newTb.dot(friction);
        }

        private void calculateTangents() {
            Vector3f tangentA = new SimpleVector3f();
            Vector3f tangentB = new SimpleVector3f();
            computeTangents(tangentA, tangentB, contactPoint.normal, contactPoint.getRelativeVelocityVec());
            projectsTangentImpulses(tangentA, tangentB);

            Vector3f ra = contactPoint.rA;
            Vector3f rb = contactPoint.rB;

            Matrix3x3 iInvA = a.getIInv();
            Matrix3x3 iInvB = b.getIInv();


            Vector3f inertiaATa = ra.duplicate().cross(tangentA);
            Vector3f inertiaBTa = rb.duplicate().cross(tangentA);

            Vector3f inertiaATb = ra.duplicate().cross(tangentB);
            Vector3f inertiaBTb = rb.duplicate().cross(tangentB);

            iInvA.transform(inertiaATa);
            iInvB.transform(inertiaBTa);
            iInvA.transform(inertiaATb);
            iInvB.transform(inertiaBTb);
            inertiaATa.cross(ra.duplicate());
            inertiaBTa.cross(rb.duplicate());
            inertiaATb.cross(ra.duplicate());
            inertiaBTb.cross(rb.duplicate());

            float angularEffectTA = inertiaATa.add(inertiaBTa).dot(tangentA);
            float angularEffectTB = inertiaATb.add(inertiaBTb).dot(tangentB);
            this.kTa = (totalMass + angularEffectTA);
            this.kTb = (totalMass + angularEffectTB);

            this.tangentA.copy(tangentA);
            this.tangentB.copy(tangentB);
        }

        public void preStep() {
            calculateTangents();
            Vector3f normal = contactPoint.normal;

            Vector3f ra = contactPoint.rA;
            Vector3f rb = contactPoint.rB;

            Vector3f fullImpulse = normal.duplicate().mul(accumulatedImpulse).add(tangentA.duplicate().mul(accumulatedFrictionA)).add(tangentB.duplicate().mul(accumulatedFrictionB));

            if (sleepB.isAwoken()) {
                b.addLinearImpulse(fullImpulse);
                b.addAngularImpulse(rb.duplicate().cross(fullImpulse.duplicate()));
            }

            if (sleepA.isAwoken()) {
                a.addLinearImpulse(fullImpulse.duplicate().negate());
                a.addAngularImpulse(ra.duplicate().cross(fullImpulse.duplicate().negate()));
            }
        }

        public float correct() {
            computeVel();

            float impulseForce = contactPoint.getRelativeVelocityAlongNormal();

            Vector3f normal = contactPoint.normal;

            Vector3f ra = contactPoint.rA;
            Vector3f rb = contactPoint.rB;

            //float epsilon = (a.getBodyProperties().bouncingFactor * b.getBodyProperties().bouncingFactor);
            float bias = (BIAS ? velBias : 0);
            /*
            if (impulseForce<-MIN_BOUNCE_VELOCITY) {
                float ve = epsilon*impulseForce;
                if (ve<-bias) {
                    bias = ve;
                }
            }

             */
            float numeratorNormal = -impulseForce + bias;

            float jr = numeratorNormal / kN;

            float oldAccumulatedImpulse = accumulatedImpulse;
            accumulatedImpulse = Math.max(jr+oldAccumulatedImpulse, 0);
            float diffJr = accumulatedImpulse-oldAccumulatedImpulse;
            totalImpulse+=accumulatedImpulse;

            float jFrictionA = -(tangentA.dot(contactPoint.getRelativeVelocityVec())) / kTa;
            float jFrictionB = -(tangentB.dot(contactPoint.getRelativeVelocityVec())) / kTb;

            float uk = a.getBodyProperties().kineticFrictionFactor * b.getBodyProperties().kineticFrictionFactor;
            float us = (a.getBodyProperties().staticFrictionFactor + b.getBodyProperties().staticFrictionFactor)*0.5f;
            float sT = (a.getBodyProperties().staticFrictionThreshold + b.getBodyProperties().staticFrictionThreshold);
            float js = sT * accumulatedImpulse;

            float jfa = abs(jFrictionA) < js ? us : uk;
            float jfb = abs(jFrictionB) < js ? us : uk;

            float oldAccumulatedFrictionA = accumulatedFrictionA;
            accumulatedFrictionA = Maths.clamp(jFrictionA + oldAccumulatedFrictionA, -jfa*accumulatedImpulse, jfa*accumulatedImpulse);
            float differenceFrictionA = accumulatedFrictionA -oldAccumulatedFrictionA;

            float oldAccumulatedFrictionB = accumulatedFrictionB;
            accumulatedFrictionB = Maths.clamp(jFrictionB + oldAccumulatedFrictionB, -jfb*accumulatedImpulse, jfb*accumulatedImpulse);
            float differenceFrictionB = accumulatedFrictionB -oldAccumulatedFrictionB;

            Vector3f fullImpulse = normal.duplicate().mul(diffJr).add(tangentA.duplicate().mul(differenceFrictionA)).add(tangentB.duplicate().mul(differenceFrictionB));

            if (sleepB.isAwoken()) {
                b.addLinearImpulse(fullImpulse);
                b.addAngularImpulse(rb.duplicate().cross(fullImpulse.duplicate()));
            }

            if (sleepA.isAwoken()) {
                a.addLinearImpulse(fullImpulse.duplicate().negate());
                a.addAngularImpulse(ra.duplicate().cross(fullImpulse.duplicate().negate()));
            }

            return abs(diffJr);
        }

        private void computeVel() {
            contactPoint.computeRealVelocity(!sleepA.isAwoken(), !sleepB.isAwoken());
        }

        public void applyBounce() {
            float epsilon = (a.getBodyProperties().bouncingFactor * b.getBodyProperties().bouncingFactor);
            if (epsilon==0)
                return;
            if (initialRelativeVelocity>-MIN_BOUNCE_VELOCITY || totalImpulse==0)
                return;

            computeVel();

            Vector3f normal = contactPoint.normal;
            Vector3f ra = contactPoint.rA;
            Vector3f rb = contactPoint.rB;

            float vn = contactPoint.getRelativeVelocityAlongNormal();
            float impulse = -(vn + epsilon*initialRelativeVelocity)/kN;

            float oldAccumulatedImpulse = accumulatedImpulse;
            accumulatedImpulse = Math.max(impulse+oldAccumulatedImpulse, 0);
            float diffJr = accumulatedImpulse-oldAccumulatedImpulse;
            totalImpulse+=diffJr;

            Vector3f fullImpulse = normal.duplicate().mul(diffJr);

            if (sleepB.isAwoken()) {
                b.addLinearImpulse(fullImpulse);
                b.addAngularImpulse(rb.duplicate().cross(fullImpulse.duplicate()));
            }

            if (sleepA.isAwoken()) {
                a.addLinearImpulse(fullImpulse.duplicate().negate());
                a.addAngularImpulse(ra.duplicate().cross(fullImpulse.duplicate().negate()));
            }
        }

        public void wakeIfNeeded() {
            if (WAKE_UP_MANUALLY && (a.getLinearVelocity().lengthSquared() >= WAKE_THRESHOLD*WAKE_THRESHOLD || b.getLinearVelocity().lengthSquared() >= WAKE_THRESHOLD*WAKE_THRESHOLD)) {
                sleepA.wakeUp();
                sleepB.wakeUp();
            }

            contactPoint.manifoldPoint.data.oldTa.copy(tangentA);
            contactPoint.manifoldPoint.data.oldTb.copy(tangentB);
            contactPoint.manifoldPoint.data.cachedAccumulatedImpulse = accumulatedImpulse;
            contactPoint.manifoldPoint.data.cachedAccumulatedFrictionLambda[0] = accumulatedFrictionA;
            contactPoint.manifoldPoint.data.cachedAccumulatedFrictionLambda[1] = accumulatedFrictionB;

            contactPoint.manifoldPoint.data.cachedTotalMass = totalMass;

        }

        public void zeroIfSleeping() {
            if (!sleepA.isAwoken()) {
                a.zeroAngularMomentum();
                a.zeroLinearMomentum();
            }
            if (!sleepB.isAwoken()) {
                b.zeroAngularMomentum();
                b.zeroLinearMomentum();
            }
        }
    }

}
