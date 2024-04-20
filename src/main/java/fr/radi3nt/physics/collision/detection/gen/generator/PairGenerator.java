package fr.radi3nt.physics.collision.detection.gen.generator;

import fr.radi3nt.physics.collision.contact.ContactPair;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.List;

public interface PairGenerator {

    void pair(List<ContactPair> pairList, RigidBody a, RigidBody b);

}
