package fr.radi3nt.physics.collision.response;

import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;

import java.util.Collection;

public interface CollisionContactSolver {
    void solve(Collection<PersistentManifold> manifold, float dt);
}
