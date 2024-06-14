package fr.radi3nt.physics.collision.response.constrained;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.collision.contact.manifold.contact.ContactPoint;
import fr.radi3nt.physics.constraints.constraint.Constraint;
import fr.radi3nt.physics.constraints.constraint.DriftParameters;
import fr.radi3nt.physics.constraints.constraint.caching.provider.ArrayCachingModuleProvider;
import fr.radi3nt.physics.constraints.constraint.constraints.FrictionConstraint;
import fr.radi3nt.physics.constraints.constraint.constraints.NoPenetrationConstraint;
import fr.radi3nt.physics.constraints.constraint.index.ImplicitRigidBodyIndex;
import fr.radi3nt.physics.constraints.constraint.index.RigidBodyIndex;

import java.util.Collection;

import static java.lang.Math.abs;

public class SimpleNoPenetrationConstraintProvider implements NoPenetrationConstraintProvider {

    private static final Vector3f[] AXIS = new Vector3f[] {
            new SimpleVector3f(1, 0, 0),
            new SimpleVector3f(0, 1, 0),
            new SimpleVector3f(0, 0, 1),
    };
    private final DriftParameters driftParameters;

    public SimpleNoPenetrationConstraintProvider(DriftParameters driftParameters) {
        this.driftParameters = driftParameters;
    }

    @Override
    public void addConstraint(Collection<Constraint> constraints, PersistentManifold persistentManifold, ContactPoint contactPoint) {
        RigidBodyIndex indexA = new ImplicitRigidBodyIndex(persistentManifold.getObjectA());
        RigidBodyIndex indexB = new ImplicitRigidBodyIndex(persistentManifold.getObjectB());
        constraints.add(new NoPenetrationConstraint(driftParameters, contactPoint.normal, contactPoint.rA, contactPoint.rB, indexA, indexB, new ArrayCachingModuleProvider(new NoPenetrationCachingConstraintModule(contactPoint.manifoldPoint))));

        Vector3f randomVec = chooseAxis(contactPoint.normal);
        if (randomVec==null)
            return;
        Vector3f tangent = contactPoint.normal.duplicate().cross(randomVec).normalize();
        Vector3f otherTangent = contactPoint.normal.duplicate().cross(tangent);
        float uk = contactPoint.a.getBodyProperties().kineticFrictionFactor*contactPoint.b.getBodyProperties().kineticFrictionFactor;
        float relativeVel = contactPoint.manifoldPoint.cachedContactLambda;
        if (relativeVel!=0 && uk!=0)
            constraints.add(new FrictionConstraint(tangent, otherTangent, relativeVel, uk, contactPoint.rA, contactPoint.rB, indexA, indexB, new ArrayCachingModuleProvider(new FrictionCachingConstraintModule(contactPoint.manifoldPoint, 0), new FrictionCachingConstraintModule(contactPoint.manifoldPoint, 1))));
    }

    private Vector3f chooseAxis(Vector3f contactNormal) {
        Vector3f random = AXIS[0];
        int i = 1;
        while (abs(random.dot(contactNormal))>=0.99f) {
            if (i>=AXIS.length)
                return null;
            random = AXIS[i];
            i++;
        }
        return random;
    }
}
