package fr.radi3nt.physics.constraints.solver.mass;

import fr.radi3nt.maths.components.advanced.matrix.Matrix3x3;
import fr.radi3nt.physics.constraints.constraint.index.RigidBodyIndex;
import fr.radi3nt.physics.core.state.DynamicsData;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;
import fr.radi3nt.physics.math.ArbitraryMatrix;
import fr.radi3nt.physics.math.matrices.array.FlatArrayArbitraryMatrix;

import static fr.radi3nt.physics.constraints.constraint.StateConstraint.STATE_STRIDE;

public class ArrayInverseMassMatrixComputer implements InverseMassMatrixComputer {
    @Override
    public ArbitraryMatrix computeInverseMassMatrix(DynamicsData[] bodies) {
        FlatArrayArbitraryMatrix matrix = new FlatArrayArbitraryMatrix(bodies.length*STATE_STRIDE, bodies.length*STATE_STRIDE);
        for (int i = 0; i < bodies.length; i++) {
            DynamicsData data = bodies[i];
            int inverseMassIndex = i*STATE_STRIDE;

            float inverseMass = data.getBodyProperties().inverseMass;
            matrix.set(inverseMassIndex, inverseMassIndex, inverseMass);
            matrix.set(inverseMassIndex+1, inverseMassIndex+1, inverseMass);
            matrix.set(inverseMassIndex+2, inverseMassIndex+2, inverseMass);

            int inertiaTensorIndex = inverseMassIndex + 3;
            Matrix3x3 inertiaTensor = data.getIInv();
            for (int x = 0; x < 3; x++) {
                for (int y = 0; y < 3; y++) {
                    matrix.set(inertiaTensorIndex + x, inertiaTensorIndex + y, inertiaTensor.get(x, y));
                }
            }
        }
        return matrix;
    }
}
