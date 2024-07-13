package fr.radi3nt.physics.math.matrices;

import fr.radi3nt.physics.math.ArbitraryMatrix;

public interface EditableArbitraryMatrix extends ArbitraryMatrix {

    void set(int x, int y, float set);
    void add(int x, int y, float value);

}
