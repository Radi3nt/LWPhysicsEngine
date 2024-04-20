package fr.radi3nt.physics.math;

import fr.radi3nt.maths.components.arbitrary.VectorNf;

public interface ArbitraryMatrix {

    float get(int x, int y);
    VectorNf transform(VectorNf vector);
    VectorNf transformTransposed(VectorNf vector);

    int getWidth();
    int getHeight();

}
