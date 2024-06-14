package fr.radi3nt.physics.collision.detection.gen;

import fr.radi3nt.physics.collision.contact.cache.ContactPairCache;
import fr.radi3nt.physics.collision.contact.cache.ContactPairCacheProvider;
import fr.radi3nt.physics.collision.contact.cache.ListContactPairCache;
import fr.radi3nt.physics.collision.detection.gen.generator.PairGenerator;
import fr.radi3nt.physics.collision.detection.gen.provider.OneDimensionProvider;
import fr.radi3nt.physics.collision.detection.gen.tools.OneDimensionOrderer;
import fr.radi3nt.physics.collision.detection.gen.tools.OneDimensionPairMeeter;
import fr.radi3nt.physics.core.state.RigidBody;
import fr.radi3nt.physics.dynamics.island.RigidBodyIsland;

import java.util.ArrayList;
import java.util.Collection;
import java.util.function.Supplier;

public class DimensionalContactPairCacheProvider implements ContactPairCacheProvider {

    private final Supplier<RigidBodyIsland> rigidBodyIsland;
    private final PairGenerator pairGenerator;
    private final OneDimensionProvider oneDimensionProvider;

    public DimensionalContactPairCacheProvider(Supplier<RigidBodyIsland> rigidBodyIsland, PairGenerator pairGenerator, OneDimensionProvider oneDimensionProvider) {
        this.rigidBodyIsland = rigidBodyIsland;
        this.pairGenerator = pairGenerator;
        this.oneDimensionProvider = oneDimensionProvider;
    }

    @Override
    public ContactPairCache newFilledCache() {
        OneDimensionPairMeeter oneDimensionPairMeeter = new OneDimensionPairMeeter(pairGenerator);
        OneDimensionOrderer oneDimensionOrderer = new OneDimensionOrderer(oneDimensionProvider);

        Collection<RigidBody> rigidBodies = new ArrayList<>();
        Collection<RigidBody> nonPhysical = new ArrayList<>();

        RigidBodyIsland island = rigidBodyIsland.get();
        for (int i = 0; i < island.getSize(); i++) {
            RigidBody rigidBody = island.getRigidBody(i);
            if (rigidBody.getCollisionData().getPreCollisionShape()!=null) {
                rigidBodies.add(rigidBody);
            } else {
                nonPhysical.add(rigidBody);
            }
        }
        oneDimensionOrderer.sort(rigidBodies);
        for (RigidBody rigidBody : nonPhysical) {
            oneDimensionPairMeeter.add(rigidBody);
        }
        for (OneDimensionOrderer.OneDimensionBody sortedOneDimensionBody : oneDimensionOrderer.getSortedOneDimensionBodies()) {
            if (sortedOneDimensionBody.isEnd()) {
                oneDimensionPairMeeter.remove(sortedOneDimensionBody.getRigidBody());
            } else {
                oneDimensionPairMeeter.add(sortedOneDimensionBody.getRigidBody());
            }
        }

        return new ListContactPairCache(oneDimensionPairMeeter.getPairs());
    }

}
