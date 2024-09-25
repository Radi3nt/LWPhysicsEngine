package fr.radi3nt.physics.collision.response.constrained;

import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.collision.contact.manifold.contact.ContactPoint;
import fr.radi3nt.physics.collision.response.CollisionContactSolver;
import fr.radi3nt.physics.constraints.constraint.Constraint;
import fr.radi3nt.physics.constraints.constraint.list.InstantConstraintList;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.ArrayList;
import java.util.Collection;

public class SimultaneousImpulseRestingContactSolver implements CollisionContactSolver {

    private final NoPenetrationConstraintProvider noPenetrationConstraintProvider;
    private final InstantConstraintList instantConstraintList;

    public SimultaneousImpulseRestingContactSolver(NoPenetrationConstraintProvider noPenetrationConstraintProvider, InstantConstraintList instantConstraintList) {
        this.noPenetrationConstraintProvider = noPenetrationConstraintProvider;
        this.instantConstraintList = instantConstraintList;
    }

    @Override
    public void solve(Collection<PersistentManifold> manifold, float dt) {
        Collection<Constraint> constraints = new ArrayList<>();
        for (PersistentManifold persistentManifold : manifold) {
            RigidBody bodyA = persistentManifold.getObjectA();
            RigidBody bodyB = persistentManifold.getObjectB();

            for (ContactPoint contactPoint : persistentManifold.getContactPoints(bodyA.getDynamicsData(), bodyB.getDynamicsData())) {
                contactPoint.computeRealVelocity();
                noPenetrationConstraintProvider.addConstraint(constraints, persistentManifold, contactPoint);
            }
        }
        instantConstraintList.getStepConstraints().addAll(constraints);
    }
}
