package fr.radi3nt.physics.collision.detection.gen.tools;

import fr.radi3nt.physics.collision.contact.ContactPair;
import fr.radi3nt.physics.collision.detection.gen.generator.PairGenerator;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

public class OneDimensionPairMeeter {

    private final List<RigidBody> activeList = new ArrayList<>();
    private final List<ContactPair> pairs = new ArrayList<>();

    private final PairGenerator pairGenerator;
    public OneDimensionPairMeeter(PairGenerator pairGenerator) {
        this.pairGenerator = pairGenerator;
    }

    public void reset() {
        pairs.clear();
        activeList.clear();
    }

    public void add(RigidBody rigidBody) {
        generatePairsContacts(rigidBody);
        activeList.add(rigidBody);
    }

    public void remove(RigidBody rigidBody) {
        activeList.remove(rigidBody);
    }

    private void generatePairsContacts(RigidBody dimensionBody) {
        for (RigidBody body : activeList) {
            pairGenerator.pair(this.pairs, body, dimensionBody);
        }
    }

    public List<ContactPair> getPairs() {
        return pairs;
    }
}
