package fr.radi3nt.physics.core;

import java.util.Vector;

public class VectorObjectArray<T> extends ListObjectArray<T> {

    public VectorObjectArray() {
        super(new Vector<>());
    }
}
