package fr.radi3nt.physics.constraints.solver.filled;

import fr.radi3nt.maths.components.arbitrary.VectorNf;
import fr.radi3nt.physics.constraints.constraint.caching.CachingConstraintModule;
import fr.radi3nt.physics.constraints.constraint.index.RigidBodyIndex;
import fr.radi3nt.physics.core.state.DynamicsData;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.math.ArbitraryMatrix;

public class FilledData {

    public final DynamicsData[] rigidBodiesIndex;
    public final CachingConstraintModule[] constraintModules;

    public final ArbitraryMatrix j;
    public final VectorNf min;
    public final VectorNf max;
    public final VectorNf ks;

    public final float[] c;

    public FilledData(DynamicsData[] rigidBodiesIndex, CachingConstraintModule[] constraintModules, ArbitraryMatrix j, VectorNf min, VectorNf max, VectorNf ks, float[] c) {
        this.rigidBodiesIndex = rigidBodiesIndex;
        this.constraintModules = constraintModules;
        this.j = j;
        this.min = min;
        this.max = max;
        this.ks = ks;
        this.c = c;
    }
}
