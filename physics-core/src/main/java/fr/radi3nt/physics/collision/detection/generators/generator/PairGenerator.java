package fr.radi3nt.physics.collision.detection.generators.generator;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.shape.pre.PreCollisionData;
import fr.radi3nt.physics.core.TransformedObject;

import java.util.List;

public interface PairGenerator<T extends TransformedObject> {

    void pair(List<GeneratedContactPair<T>> pairList, T a, T b, PreCollisionData aData, PreCollisionData bData);

}
