package fr.radi3nt.physics.core.state.inertia;

import fr.radi3nt.maths.components.advanced.matrix.Matrix3x3;

public interface InertiaTensor {

    Matrix3x3 getInverseTensor();

}
