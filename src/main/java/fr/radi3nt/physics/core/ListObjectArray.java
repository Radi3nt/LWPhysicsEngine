package fr.radi3nt.physics.core;

import java.util.List;

public class ListObjectArray<T> implements ObjectArray<T> {

    private final List<T> vector;

    public ListObjectArray(List<T> vector) {
        this.vector = vector;
    }

    @Override
    public T get(int i) {
        return vector.get(i);
    }

    @Override
    public void add(T adding) {
        vector.add(adding);
    }

    @Override
    public void remove(T adding) {
        vector.remove(adding);
    }
}
