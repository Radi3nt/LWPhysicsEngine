package fr.radi3nt.physics.collision.contact.manifold.contact;

public enum ContactType {

    SEPARATING,
    RESTING,
    COLLIDING;


    public static ContactType fromDot(float dot, float interval) {
        if (dot < -interval) {
            return ContactType.COLLIDING;
        }
        if (dot <= interval) {
            return ContactType.RESTING;
        }

        return ContactType.SEPARATING;
    }
}
