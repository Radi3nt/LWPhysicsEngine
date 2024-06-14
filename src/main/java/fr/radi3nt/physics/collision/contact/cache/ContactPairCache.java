package fr.radi3nt.physics.collision.contact.cache;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;

import java.util.ListIterator;

public interface ContactPairCache extends Iterable<GeneratedContactPair> {
    void remove(int index);
    GeneratedContactPair[] collection();
    ListIterator<GeneratedContactPair> iterator();

    int size();
}
