package fr.radi3nt.physics.collision.detection.generators;

import fr.radi3nt.physics.collision.contact.cache.ContactPairCache;
import fr.radi3nt.physics.collision.contact.cache.ContactPairCacheProvider;
import fr.radi3nt.physics.collision.contact.cache.ListContactPairCache;
import fr.radi3nt.physics.collision.detection.generators.generator.PairGenerator;
import fr.radi3nt.physics.collision.detection.generators.provider.OneDimensionProvider;
import fr.radi3nt.physics.collision.detection.generators.tools.OneDimensionOrderer;
import fr.radi3nt.physics.collision.detection.generators.tools.OneDimensionPairMeeter;
import fr.radi3nt.physics.collision.detection.generators.tools.RigidBodyOneDimensionPairMeeter;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

import java.util.ArrayList;
import java.util.Collection;

public class DimensionalContactPairCacheProvider implements ContactPairCacheProvider {

    private final PairGenerator<RigidBody> pairGenerator;
    private final OneDimensionProvider oneDimensionProvider;

    public DimensionalContactPairCacheProvider(PairGenerator<RigidBody> pairGenerator, OneDimensionProvider oneDimensionProvider) {
        this.pairGenerator = pairGenerator;
        this.oneDimensionProvider = oneDimensionProvider;
    }

    @Override
    public ContactPairCache<RigidBody> newFilledCache(RigidBodyIsland island) {
        OneDimensionPairMeeter<RigidBody> oneDimensionPairMeeter = new RigidBodyOneDimensionPairMeeter(pairGenerator);
        OneDimensionOrderer<RigidBody> oneDimensionOrderer = new OneDimensionOrderer<>(oneDimensionProvider);

        Collection<OneDimensionPairMeeter.StoredBody<RigidBody>> rigidBodies = new ArrayList<>();
        Collection<OneDimensionPairMeeter.StoredBody<RigidBody>> nonPhysical = new ArrayList<>();

        for (int i = 0; i < island.getSize(); i++) {
            RigidBody rigidBody = island.getRigidBody(i);
            if (rigidBody.getCollisionData().getPreCollisionShape()!=null) {
                rigidBodies.add(new OneDimensionPairMeeter.StoredBody<>(rigidBody, rigidBody.getCollisionData().getPreCollisionShape().toData(rigidBody)));
            } else {
                nonPhysical.add(new OneDimensionPairMeeter.StoredBody<>(rigidBody, null));
            }
        }
        oneDimensionOrderer.sort(rigidBodies);
        for (OneDimensionPairMeeter.StoredBody<RigidBody> rigidBody : nonPhysical) {
            oneDimensionPairMeeter.add(rigidBody);
        }
        for (OneDimensionOrderer.OneDimensionBody<RigidBody> sortedOneDimensionBody : oneDimensionOrderer.getSortedOneDimensionBodies()) {
            if (sortedOneDimensionBody.isEnd()) {
                oneDimensionPairMeeter.remove(sortedOneDimensionBody.getRigidBody());
            } else {
                oneDimensionPairMeeter.add(sortedOneDimensionBody.getRigidBody());
            }
        }

        return new ListContactPairCache<>(oneDimensionPairMeeter.getPairs());
    }

}
