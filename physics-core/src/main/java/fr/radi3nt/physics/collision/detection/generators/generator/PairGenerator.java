package fr.radi3nt.physics.collision.detection.generators.generator;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.shape.pre.PreCollisionData;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.List;

public interface PairGenerator {

    void pair(List<GeneratedContactPair> pairList, RigidBody a, RigidBody b, PreCollisionData aData, PreCollisionData bData);

}
