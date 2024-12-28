package fr.radi3nt.physics.collision.contact.cache;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.core.TransformedObject;

import java.util.ListIterator;

public interface ContactPairCache<T extends TransformedObject> extends Iterable<GeneratedContactPair<T>> {
    void remove(int index);
    ListIterator<GeneratedContactPair<T>> iterator();

    int size();
}
