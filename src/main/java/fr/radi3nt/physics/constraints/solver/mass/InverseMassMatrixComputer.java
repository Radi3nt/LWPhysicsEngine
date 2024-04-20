package fr.radi3nt.physics.constraints.solver.mass;

import fr.radi3nt.physics.constraints.constraint.index.RigidBodyIndex;
import fr.radi3nt.physics.core.state.DynamicsData;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;
import fr.radi3nt.physics.math.ArbitraryMatrix;

public interface InverseMassMatrixComputer {

    ArbitraryMatrix computeInverseMassMatrix(DynamicsData[] bodies);

}
