package fr.radi3nt.physics.collision.response.constrained;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.collision.contact.manifold.contact.ContactPoint;
import fr.radi3nt.physics.collision.response.sequential.SequentialImpulseCollisionContactSolver;
import fr.radi3nt.physics.constraints.constraint.Constraint;
import fr.radi3nt.physics.constraints.constraint.DriftParameters;
import fr.radi3nt.physics.constraints.constraint.caching.provider.ArrayCachingModuleProvider;
import fr.radi3nt.physics.constraints.constraint.constraints.FrictionConstraint;
import fr.radi3nt.physics.constraints.constraint.constraints.NoPenetrationConstraint;
import fr.radi3nt.physics.constraints.constraint.index.ImplicitRigidBodyIndex;
import fr.radi3nt.physics.constraints.constraint.index.RigidBodyIndex;

import java.util.Collection;

public class SimpleNoPenetrationConstraintProvider implements NoPenetrationConstraintProvider {

    private static final float VELOCITY_THRESHOLD = 1E-1f;
    private final DriftParameters driftParameters;

    public SimpleNoPenetrationConstraintProvider(DriftParameters driftParameters) {
        this.driftParameters = driftParameters;
    }

    @Override
    public void addConstraint(Collection<Constraint> constraints, PersistentManifold persistentManifold, ContactPoint contactPoint) {
        if (!contactPoint.isRestingContact())
            return;

        RigidBodyIndex indexA = new ImplicitRigidBodyIndex(persistentManifold.getObjectA());
        RigidBodyIndex indexB = new ImplicitRigidBodyIndex(persistentManifold.getObjectB());
        constraints.add(new NoPenetrationConstraint(driftParameters, contactPoint.normal, contactPoint.rA, contactPoint.rB, indexA, indexB, new ArrayCachingModuleProvider(new NoPenetrationCachingConstraintModule(contactPoint.manifoldPoint.data))));

        Vector3f tangent = new SimpleVector3f();
        Vector3f otherTangent = new SimpleVector3f();
        SequentialImpulseCollisionContactSolver.computeTangents(tangent, otherTangent, contactPoint.normal, contactPoint.getRelativeVelocityVec());

        float uk = contactPoint.a.getBodyProperties().kineticFrictionFactor * contactPoint.b.getBodyProperties().kineticFrictionFactor;
        float relativeVel = contactPoint.manifoldPoint.data.cachedContactLambda;
        if (relativeVel*uk > VELOCITY_THRESHOLD)
            constraints.add(new FrictionConstraint(tangent,
                    otherTangent,
                    relativeVel,
                    uk,
                    contactPoint.rA,
                    contactPoint.rB,
                    indexA,
                    indexB,
                    new ArrayCachingModuleProvider(new FrictionCachingConstraintModule(contactPoint.manifoldPoint.data, 0),
                            new FrictionCachingConstraintModule(contactPoint.manifoldPoint.data, 1))));
    }
}
