package fr.radi3nt.physics.constraints.solver.mass;

import fr.radi3nt.maths.components.advanced.matrix.Matrix3x3;
import fr.radi3nt.physics.constraints.constraint.index.IdentifiedDynamicsData;
import fr.radi3nt.physics.constraints.solver.filled.FilledData;
import fr.radi3nt.physics.core.state.DynamicsData;
import fr.radi3nt.physics.math.ArbitraryMatrix;
import fr.radi3nt.physics.math.matrices.sparse.HashSparseArbitraryMatrix;
import fr.radi3nt.physics.math.matrices.sparse.SparseBlock;

import static fr.radi3nt.physics.constraints.constraint.StateConstraint.STATE_STRIDE;

public class SparseInverseMassMatrixComputer implements InverseMassMatrixComputer {
    @Override
    public ArbitraryMatrix computeInverseMassMatrix(FilledData filledData) {
        IdentifiedDynamicsData[] bodies = filledData.rigidBodiesIndex;
        HashSparseArbitraryMatrix matrix = new HashSparseArbitraryMatrix();
        for (int i = 0; i < bodies.length; i++) {
            IdentifiedDynamicsData idData = bodies[i];
            DynamicsData data = idData.data;
            int inverseMassIndex = i*STATE_STRIDE;

            float mul = idData.isStatic() ? 0 : 1;

            float inverseMass = data.getBodyProperties().inverseMass*mul;
            SparseBlock iMass = new SparseBlock(inverseMassIndex, inverseMassIndex, 3, 3, new float[] {
                    inverseMass, 0, 0,
                    0, inverseMass, 0,
                    0, 0, inverseMass
            });
            matrix.add(iMass);


            int inertiaTensorIndex = inverseMassIndex + 3;
            float[] values = new float[3*3];

            Matrix3x3 inertiaTensor = data.getIInv();
            for (int x = 0; x < 3; x++) {
                for (int y = 0; y < 3; y++) {
                    values[x+y*3]=inertiaTensor.get(x, y)*mul;
                }
            }
            SparseBlock tensor = new SparseBlock(inertiaTensorIndex, inertiaTensorIndex, 3, 3, values);
            matrix.add(tensor);
        }
        matrix.calcSet();
        return matrix;
    }
}
