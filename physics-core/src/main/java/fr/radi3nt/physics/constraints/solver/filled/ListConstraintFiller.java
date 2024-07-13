package fr.radi3nt.physics.constraints.solver.filled;

import fr.radi3nt.maths.components.arbitrary.VectorNf;
import fr.radi3nt.maths.components.arbitrary.vector.ExtensibleVectorNf;
import fr.radi3nt.physics.constraints.constraint.Constraint;
import fr.radi3nt.physics.constraints.constraint.ConstraintData;
import fr.radi3nt.physics.constraints.constraint.DriftParameters;
import fr.radi3nt.physics.constraints.constraint.StateConstraint;
import fr.radi3nt.physics.constraints.constraint.caching.CachingConstraintModule;
import fr.radi3nt.physics.constraints.constraint.caching.provider.CachingModuleProvider;
import fr.radi3nt.physics.constraints.constraint.index.IdentifiedDynamicsData;
import fr.radi3nt.physics.constraints.constraint.list.ConstraintList;
import fr.radi3nt.physics.core.state.DynamicsData;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;
import fr.radi3nt.physics.math.matrices.sparse.SparseArbitraryMatrix;
import fr.radi3nt.physics.math.matrices.sparse.SparseBlock;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import static fr.radi3nt.physics.constraints.constraint.StateConstraint.STATE_STRIDE;

public class ListConstraintFiller implements ConstraintFiller {

    private final ConstraintList constraints;

    public ListConstraintFiller(ConstraintList constraints) {
        this.constraints = constraints;
    }

    @Override
    public FilledData fill(RigidBodyIsland island) {
        SparseArbitraryMatrix sparseArbitraryMatrix = new SparseArbitraryMatrix();
        ExtensibleVectorNf min = new ExtensibleVectorNf();
        ExtensibleVectorNf max = new ExtensibleVectorNf();
        ExtensibleVectorNf ks = new ExtensibleVectorNf();
        ExtensibleVectorNf c = new ExtensibleVectorNf();

        int constraintIndex = 0;

        List<IdentifiedDynamicsData> concernedBodies = new ArrayList<>();
        List<CachingConstraintModule> constraintModules = new ArrayList<>();

        Collection<Constraint> currentConstraints = constraints.getConstraints();
        for (Constraint constraint : currentConstraints) {
            ConstraintData constraintData = constraint.compute(island);
            if (constraintData==null)
                continue;

            CachingModuleProvider modules = constraintData.getCachingConstraintModules();
            DriftParameters[] driftParameters = constraintData.getDriftParameters();
            float[] currentMin = constraintData.getMin();
            float[] currentMax = constraintData.getMax();
            float[] currentC = constraintData.getCorrections();

            StateConstraint[] impulses = constraintData.getImpulses();
            for (int localConstraintIndex = 0, impulsesLength = impulses.length; localConstraintIndex < impulsesLength; localConstraintIndex++) {
                StateConstraint impulse = impulses[localConstraintIndex];
                IdentifiedDynamicsData[] bodies = impulse.getConcernedBodies();
                for (int localBodyIndex = 0, bodiesLength = bodies.length; localBodyIndex < bodiesLength; localBodyIndex++) {
                    IdentifiedDynamicsData concernedBody = bodies[localBodyIndex];
                    int sparseIndex = regularIndexToMatrixIndex(concernedBodies, concernedBody);
                    VectorNf states = impulse.getStates()[localBodyIndex];
                    sparseArbitraryMatrix.add(new SparseBlock(sparseIndex * STATE_STRIDE, constraintIndex, STATE_STRIDE, 1, vectorToArray(states)));
                }

                min.add(currentMin[localConstraintIndex]);
                max.add(currentMax[localConstraintIndex]);
                c.add(currentC[localConstraintIndex]);

                DriftParameters currentDrift = driftParameters[localConstraintIndex];
                ks.add(currentDrift.ks);

                constraintModules.add(modules.get(localConstraintIndex));

                constraintIndex++;
            }
        }

        IdentifiedDynamicsData[] bodies = new IdentifiedDynamicsData[concernedBodies.size()];
        DynamicsData[] data = new DynamicsData[concernedBodies.size()];
        for (int i = 0; i < concernedBodies.size(); i++) {
            bodies[i] = concernedBodies.get(i);
            data[i] = concernedBodies.get(i).data;
        }

        return new FilledData(bodies, data, constraintModules.toArray(new CachingConstraintModule[0]), sparseArbitraryMatrix, min, max, ks, vectorToArray(c));
    }

    private float[] vectorToArray(VectorNf states) {
        float[] array = new float[states.size()];
        for (int i = 0; i < array.length; i++) {
            array[i] = states.get(i);
        }
        return array;
    }

    private int regularIndexToMatrixIndex(List<IdentifiedDynamicsData> concernedBodies, IdentifiedDynamicsData concernedBody) {
        for (int i = 0; i < concernedBodies.size(); i++) {
            IdentifiedDynamicsData body = concernedBodies.get(i);
            if (body.equals(concernedBody))
                return i;
        }
        int matrixIndex = concernedBodies.size();
        concernedBodies.add(concernedBody);
        return matrixIndex;
    }
}
