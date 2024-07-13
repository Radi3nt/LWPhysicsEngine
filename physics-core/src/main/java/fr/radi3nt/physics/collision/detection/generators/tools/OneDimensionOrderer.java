package fr.radi3nt.physics.collision.detection.generators.tools;

import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.mapping.AxisMapping;
import fr.radi3nt.physics.collision.detection.generators.provider.OneDimensionProvider;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class OneDimensionOrderer {

    public final List<OneDimensionBody> sortedOneDimensionBodies = new ArrayList<>();
    private final OneDimensionProvider oneDimensionProvider;

    public OneDimensionOrderer(OneDimensionProvider oneDimensionProvider) {
        this.oneDimensionProvider = oneDimensionProvider;
    }

    public void sort(Collection<RigidBody> rigidBodies) {
        for (RigidBody rigidBody : rigidBodies) {
            AxisMapping axisMapping = oneDimensionProvider.map(rigidBody, rigidBody.getCollisionData().getPreCollisionShape());
            sortedOneDimensionBodies.add(new OneDimensionBody(rigidBody, axisMapping.getMax(), true));
            sortedOneDimensionBodies.add(new OneDimensionBody(rigidBody, axisMapping.getMin(), false));
        }

        sortedOneDimensionBodies.sort((o1, o2) -> {
            int compare = Float.compare(o1.value, o2.value);
            if (compare==0)
                return Boolean.compare(o1.end, o2.end);
            return compare;
        });
    }

    public List<OneDimensionBody> getSortedOneDimensionBodies() {
        return sortedOneDimensionBodies;
    }

    public static final class OneDimensionBody {

        private final RigidBody rigidBody;
        private final float value;
        private final boolean end;

        public OneDimensionBody(RigidBody rigidBody, float value, boolean end) {
            this.rigidBody = rigidBody;
            this.value = value;
            this.end = end;
        }

        public RigidBody getRigidBody() {
            return rigidBody;
        }

        public boolean isEnd() {
            return end;
        }
    }

}
