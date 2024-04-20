package fr.radi3nt.physics.collision.contact.cache;

import fr.radi3nt.physics.collision.contact.ContactPair;
import fr.radi3nt.physics.collision.detection.gen.generator.PairGenerator;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

import java.util.*;

public class ListContactPairCache implements ContactPairCache {

    private final List<ContactPair> contactPairs;

    public ListContactPairCache(List<ContactPair> contactPairs) {
        this.contactPairs = contactPairs;
    }

    public static ListContactPairCache fromIsland(RigidBodyIsland rigidBodyIsland, PairGenerator pairGenerator) {
        List<ContactPair> pairs = new ArrayList<>();
        for (int i = 0; i < rigidBodyIsland.getSize(); i++) {
            for (int j = 0; j < rigidBodyIsland.getSize(); j++) {
                if (i>j) {
                    RigidBody rigidBodyA = rigidBodyIsland.getRigidBody(i);
                    RigidBody rigidBodyB = rigidBodyIsland.getRigidBody(j);

                    pairGenerator.pair(pairs, rigidBodyA, rigidBodyB);
                }
            }
        }
        return new ListContactPairCache(pairs);
    }


    @Override
    public void remove(int index) {
        this.contactPairs.remove(index);
    }

    @Override
    public ContactPair[] collection() {
        return contactPairs.toArray(new ContactPair[0]);
    }

    @Override
    public ListIterator<ContactPair> iterator() {
        return contactPairs.listIterator();
    }

    @Override
    public int size() {
        return contactPairs.size();
    }
}
