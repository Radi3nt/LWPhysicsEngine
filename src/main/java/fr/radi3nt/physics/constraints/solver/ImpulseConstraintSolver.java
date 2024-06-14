package fr.radi3nt.physics.constraints.solver;

import fr.radi3nt.maths.components.arbitrary.VectorNf;
import fr.radi3nt.maths.components.arbitrary.vector.ArrayVectorNf;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.constraints.constraint.caching.CachingConstraintModule;
import fr.radi3nt.physics.constraints.sle.SleSolver;
import fr.radi3nt.physics.constraints.solver.caching.ConstraintCacher;
import fr.radi3nt.physics.constraints.solver.filled.ConstraintFiller;
import fr.radi3nt.physics.constraints.solver.filled.FilledData;
import fr.radi3nt.physics.constraints.solver.mass.InverseMassMatrixComputer;
import fr.radi3nt.physics.core.state.DynamicsData;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;
import fr.radi3nt.physics.math.ArbitraryMatrix;
import fr.radi3nt.physics.math.matrices.sparse.SparseArbitraryMatrix;
import fr.radi3nt.physics.math.multiplication.algorithm.SparseMatrixMultiplicationAlgorithm;
import fr.radi3nt.physics.math.multiplication.algorithm.SparseSparseMatrixMultiplicationAlgorithm;

import static fr.radi3nt.physics.constraints.constraint.StateConstraint.STATE_STRIDE;

public class ImpulseConstraintSolver implements ConstraintSolver {

    private final InverseMassMatrixComputer inverseMassMatrixComputer;
    private final ConstraintCacher lambdaCaching;
    private final SleSolver solver;

    public ImpulseConstraintSolver(InverseMassMatrixComputer inverseMassMatrixComputer, ConstraintCacher lambdaCaching, SleSolver solver) {
        this.inverseMassMatrixComputer = inverseMassMatrixComputer;
        this.lambdaCaching = lambdaCaching;
        this.solver = solver;
    }

    @Override
    public void solve(ConstraintFiller filler, RigidBodyIsland island, float dt) {
        FilledData filledData = filler.fill(island);

        DynamicsData[] bodiesIndex = filledData.rigidBodiesIndex;
        CachingConstraintModule[] modules = filledData.constraintModules;
        ArbitraryMatrix m = inverseMassMatrixComputer.computeInverseMassMatrix(bodiesIndex);
        ArbitraryMatrix j = filledData.j;
        ArbitraryMatrix a = computeA(j, m);
        VectorNf v = computeV(bodiesIndex);
        VectorNf b = computeB(j, v);
        VectorNf computeDamping = computeDamping(filledData.ks, filledData.c, dt);
        b.add(computeDamping);
        b.mul(-1);

        lambdaCaching.prepare(modules);
        VectorNf lambda = solver.solve(a, b, filledData.min, filledData.max);
        lambdaCaching.cache(modules, lambda);

        VectorNf impulses = j.transformTransposed(lambda);
        addImpulses(bodiesIndex, impulses);
    }

    private static void addImpulses(DynamicsData[] bodiesIndex, VectorNf impulses) {
        for (int i = 0; i < bodiesIndex.length; i++) {
            DynamicsData data = bodiesIndex[i];
            Vector3f linearImpulse = new SimpleVector3f(impulses.get(i*STATE_STRIDE), impulses.get(i*STATE_STRIDE+1), impulses.get(i*STATE_STRIDE+2));
            Vector3f angularImpulse = new SimpleVector3f(impulses.get(i*STATE_STRIDE+3), impulses.get(i*STATE_STRIDE+4), impulses.get(i*STATE_STRIDE+5));
            data.addLinearImpulse(linearImpulse);
            data.addAngularImpulse(angularImpulse);
        }
    }

    private VectorNf computeDamping(VectorNf ks, float[] c, float dt) {
        VectorNf posSpringBias = computeKs(c, ks, dt);

        VectorNf result = new ArrayVectorNf(posSpringBias.size());
        for (int i = 0; i < posSpringBias.size(); i++) {
            result.set(i, (posSpringBias.get(i)));
        }

        return result;
    }

    private VectorNf computeKs(float[] c, VectorNf ks, float dt) {
        VectorNf spring = new ArrayVectorNf(c.length);
        for (int i = 0; i < c.length; i++) {
            spring.set(i, c[i]*ks.get(i)/dt);
        }
        return spring;
    }

    private VectorNf computeB(ArbitraryMatrix j, VectorNf v) {
        return j.transform(v);
    }

    private VectorNf computeV(DynamicsData[] rigidBodies) {
        VectorNf vector = new ArrayVectorNf(rigidBodies.length*STATE_STRIDE);
        for (int i = 0, rigidBodiesLength = rigidBodies.length; i < rigidBodiesLength; i++) {
            DynamicsData data = rigidBodies[i];
            Vector3f linearVelocity = data.getLinearVelocity();
            Vector3f angularVelocity = data.getAngularVelocity();

            vector.set(i * STATE_STRIDE, linearVelocity.getX());
            vector.set(i * STATE_STRIDE + 1, linearVelocity.getY());
            vector.set(i * STATE_STRIDE + 2, linearVelocity.getZ());
            vector.set(i * STATE_STRIDE + 3, angularVelocity.getX());
            vector.set(i * STATE_STRIDE + 4, angularVelocity.getY());
            vector.set(i * STATE_STRIDE + 5, angularVelocity.getZ());
        }
        return vector;
    }

    private ArbitraryMatrix computeA(ArbitraryMatrix j, ArbitraryMatrix m) {
        SparseMatrixMultiplicationAlgorithm sMul = SparseMatrixMultiplicationAlgorithm.INSTANCE;
        SparseSparseMatrixMultiplicationAlgorithm ssMul = SparseSparseMatrixMultiplicationAlgorithm.INSTANCE;
        return sMul.multiplyWithATransposedSwitchedBit((SparseArbitraryMatrix) j, ssMul.multiplyResultBit((SparseArbitraryMatrix) j, (SparseArbitraryMatrix) m));
    }

}
