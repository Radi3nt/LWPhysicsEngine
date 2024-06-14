package fr.radi3nt.physics.core.converter;

import fr.radi3nt.physics.core.TransformedObject;
import fr.radi3nt.physics.core.state.DynamicsData;
import fr.radi3nt.physics.core.state.RigidBody;

public interface CollisionObjectConverter {

    DynamicsData getData(TransformedObject object);
    RigidBody getBody(TransformedObject object);

}
