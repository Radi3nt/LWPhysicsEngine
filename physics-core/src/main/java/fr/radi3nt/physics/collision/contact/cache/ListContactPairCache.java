package fr.radi3nt.physics.collision.contact.cache;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.core.TransformedObject;

import java.util.List;
import java.util.ListIterator;

public class ListContactPairCache<T extends TransformedObject> implements ContactPairCache<T> {

    private final List<GeneratedContactPair<T>> contactPairs;

    public ListContactPairCache(List<GeneratedContactPair<T>> contactPairs) {
        this.contactPairs = contactPairs;
    }


    @Override
    public void remove(int index) {
        this.contactPairs.remove(index);
    }

    @Override
    public ListIterator<GeneratedContactPair<T>> iterator() {
        return contactPairs.listIterator();
    }

    @Override
    public int size() {
        return contactPairs.size();
    }
}
