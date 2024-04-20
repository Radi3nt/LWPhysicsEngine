package fr.radi3nt.physics.constraints.sle;

import fr.radi3nt.maths.components.arbitrary.VectorNf;
import fr.radi3nt.physics.math.ArbitraryMatrix;

public interface SleSolver {

    VectorNf solve(ArbitraryMatrix a, VectorNf b, VectorNf min, VectorNf max);

}
