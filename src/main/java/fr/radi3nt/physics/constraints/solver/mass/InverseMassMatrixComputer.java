package fr.radi3nt.physics.constraints.solver.mass;

import fr.radi3nt.physics.constraints.solver.filled.FilledData;
import fr.radi3nt.physics.math.ArbitraryMatrix;

public interface InverseMassMatrixComputer {

    ArbitraryMatrix computeInverseMassMatrix(FilledData bodies);

}
