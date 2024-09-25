package fr.radi3nt.physics.collision.contact.cache;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;

import java.util.List;
import java.util.ListIterator;

public class ListContactPairCache implements ContactPairCache {

    private final List<GeneratedContactPair> contactPairs;

    public ListContactPairCache(List<GeneratedContactPair> contactPairs) {
        this.contactPairs = contactPairs;
    }


    @Override
    public void remove(int index) {
        this.contactPairs.remove(index);
    }

    @Override
    public GeneratedContactPair[] collection() {
        return contactPairs.toArray(new GeneratedContactPair[0]);
    }

    @Override
    public ListIterator<GeneratedContactPair> iterator() {
        return contactPairs.listIterator();
    }

    @Override
    public int size() {
        return contactPairs.size();
    }
}
