package fr.radi3nt.physics.collision.response.constrained;

import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.collision.contact.manifold.contact.ContactPoint;
import fr.radi3nt.physics.collision.response.CollisionContactSolver;
import fr.radi3nt.physics.constraints.constraint.Constraint;
import fr.radi3nt.physics.constraints.constraint.list.InstantConstraintList;
import fr.radi3nt.physics.core.converter.CollisionObjectConverter;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.ArrayList;
import java.util.Collection;

public class SimultaneousImpulseRestingContactSolver implements CollisionContactSolver {

    private final CollisionObjectConverter converter;
    private final NoPenetrationConstraintProvider noPenetrationConstraintProvider;
    private final InstantConstraintList instantConstraintList;

    public SimultaneousImpulseRestingContactSolver(CollisionObjectConverter converter, NoPenetrationConstraintProvider noPenetrationConstraintProvider, InstantConstraintList instantConstraintList) {
        this.converter = converter;
        this.noPenetrationConstraintProvider = noPenetrationConstraintProvider;
        this.instantConstraintList = instantConstraintList;
    }

    @Override
    public void solve(Collection<PersistentManifold> manifold, float dt) {
        Collection<Constraint> constraints = new ArrayList<>();
        for (PersistentManifold persistentManifold : manifold) {
            RigidBody bodyA = converter.getBody(persistentManifold.getObjectA());
            RigidBody bodyB = converter.getBody(persistentManifold.getObjectB());
            if (canIgnore(bodyA) && canIgnore(bodyB))
                continue;
            for (ContactPoint contactPoint : persistentManifold.getContactPoints(bodyA.getDynamicsData(), bodyB.getDynamicsData())) {
                contactPoint.computeRealVelocity();
                if (contactPoint.isSeparatingContact())
                    continue;
                noPenetrationConstraintProvider.addConstraint(constraints, persistentManifold, contactPoint);
            }
        }
        instantConstraintList.getStepConstraints().addAll(constraints);
    }

    private static boolean canIgnore(RigidBody body) {
        return body.getSleepingData().isSleeping() || body.getDynamicsData().getBodyProperties().inverseMass==0;
    }
}
