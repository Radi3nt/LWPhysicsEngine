package fr.radi3nt.physics.collision.contact.cache;

import fr.radi3nt.physics.collision.contact.ContactPair;

import java.util.Collection;
import java.util.List;
import java.util.ListIterator;

public interface ContactPairCache extends Iterable<ContactPair> {
    void remove(int index);
    ContactPair[] collection();
    ListIterator<ContactPair> iterator();

    int size();
}
