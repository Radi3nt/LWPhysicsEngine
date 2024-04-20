package fr.radi3nt.physics.collision.response.constrained;

import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;
import fr.radi3nt.physics.collision.contact.manifold.contact.ContactPoint;
import fr.radi3nt.physics.constraints.constraint.Constraint;
import fr.radi3nt.physics.constraints.constraint.constraints.NoPenetrationConstraint;

import java.util.Collection;

public interface NoPenetrationConstraintProvider {

    void addConstraint(Collection<Constraint> constraints, PersistentManifold persistentManifold, ContactPoint contactPoint);

}
