package fr.radi3nt.physics.collision.response;

import fr.radi3nt.physics.collision.contact.manifold.PersistentManifold;

import java.util.Collection;

public class CompositionCollisionContactSolver implements CollisionContactSolver {

    private final CollisionContactSolver[] solvers;

    public CompositionCollisionContactSolver(CollisionContactSolver... solvers) {
        this.solvers = solvers;
    }

    @Override
    public void solve(Collection<PersistentManifold> manifold, float dt) {
        for (CollisionContactSolver solver : solvers) {
            solver.solve(manifold, dt);
        }
    }
}
