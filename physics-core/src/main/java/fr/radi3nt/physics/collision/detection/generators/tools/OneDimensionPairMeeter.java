package fr.radi3nt.physics.collision.detection.generators.tools;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.detection.generators.generator.PairGenerator;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.ArrayList;
import java.util.List;

public class OneDimensionPairMeeter {

    private final List<RigidBody> activeList = new ArrayList<>();
    private final List<GeneratedContactPair> pairs = new ArrayList<>();

    private final PairGenerator pairGenerator;
    public OneDimensionPairMeeter(PairGenerator pairGenerator) {
        this.pairGenerator = pairGenerator;
    }

    public void reset() {
        pairs.clear();
        activeList.clear();
    }

    public void add(RigidBody rigidBody) {
        if (rigidBody.getCollisionData().isEmpty())
            return;
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

    public List<GeneratedContactPair> getPairs() {
        return pairs;
    }
}
