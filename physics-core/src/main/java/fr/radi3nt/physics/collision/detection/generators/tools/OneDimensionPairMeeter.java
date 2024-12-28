package fr.radi3nt.physics.collision.detection.generators.tools;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.detection.generators.generator.PairGenerator;
import fr.radi3nt.physics.collision.shape.pre.PreCollisionData;
import fr.radi3nt.physics.core.TransformedObject;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public abstract class OneDimensionPairMeeter<T extends TransformedObject> {

    private final List<StoredBody<T>> activeList = new ArrayList<>();
    private final List<GeneratedContactPair<T>> pairs = new ArrayList<>();

    private final PairGenerator<T> pairGenerator;
    public OneDimensionPairMeeter(PairGenerator<T> pairGenerator) {
        this.pairGenerator = pairGenerator;
    }

    public void reset() {
        pairs.clear();
        activeList.clear();
    }

    public void add(StoredBody<T> rigidBody) {
        if (canSkip(rigidBody.body))
            return;

        generatePairsContacts(rigidBody);
        activeList.add(rigidBody);
    }

    protected abstract boolean canSkip(T rigidBody);

    public void remove(StoredBody<T> rigidBody) {
        activeList.remove(rigidBody);
    }

    private void generatePairsContacts(StoredBody<T> other) {
        for (StoredBody<T> stored : activeList) {
            pairGenerator.pair(this.pairs, stored.body, other.body, stored.data, other.data);
        }
    }

    public List<GeneratedContactPair<T>> getPairs() {
        return pairs;
    }

    public static class StoredBody<T> {

        public final T body;
        public final PreCollisionData data;

        public StoredBody(T body, PreCollisionData data) {
            this.body = body;
            this.data = data;
        }

        @Override
        public final boolean equals(Object o) {
            if (!(o instanceof StoredBody)) return false;

            StoredBody<?> that = (StoredBody<?>) o;
            return Objects.equals(body, that.body) && Objects.equals(data, that.data);
        }

        @Override
        public int hashCode() {
            int result = Objects.hashCode(body);
            result = 31 * result + Objects.hashCode(data);
            return result;
        }
    }
}
