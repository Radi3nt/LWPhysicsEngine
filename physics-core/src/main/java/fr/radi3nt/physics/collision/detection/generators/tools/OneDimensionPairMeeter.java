package fr.radi3nt.physics.collision.detection.generators.tools;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.detection.generators.generator.PairGenerator;
import fr.radi3nt.physics.collision.shape.pre.PreCollisionData;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class OneDimensionPairMeeter {

    private final List<StoredBody> activeList = new ArrayList<>();
    private final List<GeneratedContactPair> pairs = new ArrayList<>();

    private final PairGenerator pairGenerator;
    public OneDimensionPairMeeter(PairGenerator pairGenerator) {
        this.pairGenerator = pairGenerator;
    }

    public void reset() {
        pairs.clear();
        activeList.clear();
    }

    public void add(StoredBody rigidBody) {
        if (canSkip(rigidBody.body))
            return;

        generatePairsContacts(rigidBody);
        activeList.add(rigidBody);
    }

    private boolean canSkip(RigidBody rigidBody) {
        return rigidBody.getCollisionData().isEmpty();
    }

    public void remove(StoredBody rigidBody) {
        activeList.remove(rigidBody);
    }

    private void generatePairsContacts(StoredBody other) {
        for (StoredBody stored : activeList) {
            pairGenerator.pair(this.pairs, stored.body, other.body, stored.data, other.data);
        }
    }

    public List<GeneratedContactPair> getPairs() {
        return pairs;
    }

    public static class StoredBody {

        public final RigidBody body;
        public final PreCollisionData data;

        public StoredBody(RigidBody body, PreCollisionData data) {
            this.body = body;
            this.data = data;
        }

        @Override
        public final boolean equals(Object o) {
            if (this == o) return true;
            if (!(o instanceof StoredBody)) return false;

            StoredBody that = (StoredBody) o;
            return Objects.equals(body, that.body);
        }

        @Override
        public int hashCode() {
            return Objects.hashCode(body);
        }
    }
}
