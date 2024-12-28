package fr.radi3nt.physics.collision.detection.generators.tools;

import fr.radi3nt.physics.collision.detection.generators.generator.PairGenerator;
import fr.radi3nt.physics.core.state.RigidBody;

public class RigidBodyOneDimensionPairMeeter extends OneDimensionPairMeeter<RigidBody> {

    public RigidBodyOneDimensionPairMeeter(PairGenerator<RigidBody> pairGenerator) {
        super(pairGenerator);
    }

    @Override
    protected boolean canSkip(RigidBody rigidBody) {
        return rigidBody.getCollisionData().isEmpty();
    }

}
