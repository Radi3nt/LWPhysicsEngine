package fr.radi3nt.physics.collision.contact.cache;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.detection.generators.generator.PairGenerator;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;

public class ListContactPairCache implements ContactPairCache {

    private final List<GeneratedContactPair> contactPairs;

    public ListContactPairCache(List<GeneratedContactPair> contactPairs) {
        this.contactPairs = contactPairs;
    }

    public static ListContactPairCache fromIsland(RigidBodyIsland rigidBodyIsland, PairGenerator pairGenerator) {
        List<GeneratedContactPair> pairs = new ArrayList<>();
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
