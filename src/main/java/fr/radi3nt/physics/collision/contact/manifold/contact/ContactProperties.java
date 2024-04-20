package fr.radi3nt.physics.collision.contact.manifold.contact;

import fr.radi3nt.physics.core.state.DynamicsProperties;

public class ContactProperties {

    public float bouncingFactor;
    public float kineticFrictionFactor;
    public float staticFrictionFactor;

    public ContactProperties(float bouncingFactor, float kineticFrictionFactor, float staticFrictionFactor) {
        this.bouncingFactor = bouncingFactor;
        this.kineticFrictionFactor = kineticFrictionFactor;
        this.staticFrictionFactor = staticFrictionFactor;
    }

    public static ContactProperties from(DynamicsProperties a, DynamicsProperties b) {
        return new ContactProperties(a.bouncingFactor*b.bouncingFactor,
                a.kineticFrictionFactor*b.kineticFrictionFactor,
                a.staticFrictionFactor*b.staticFrictionFactor);
    }
}
