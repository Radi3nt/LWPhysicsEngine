package fr.radi3nt.physics.collision.contact.manifold;

import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;

public class ManifoldPointData {

    public Vector3f oldTa = new SimpleVector3f();
    public Vector3f oldTb = new SimpleVector3f();
    public float cachedAccumulatedImpulse;
    public float[] cachedAccumulatedFrictionLambda = new float[2];

    public float cachedTotalMass;
    public float cachedContactLambda;
    public float[] cachedFrictionLambda = new float[2];

}
