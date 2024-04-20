package fr.radi3nt.physics.collision.detection.predicate;

import fr.radi3nt.maths.aabb.AABB;
import fr.radi3nt.maths.aabb.AxisMapping;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.physics.core.state.RigidBody;

import java.util.function.Predicate;

public class RadiusPredicate implements Predicate<RigidBody> {

    private final Vector3f offset;
    private final float radius;

    public RadiusPredicate(Vector3f offset, float radius) {
        this.offset = offset;
        this.radius = radius;
    }

    @Override
    public boolean test(RigidBody rigidBody) {
        Vector3f position = rigidBody.getPosition();
        Vector3f newPos = position.duplicate().sub(offset);

        return newPos.lengthSquared()<=radius*radius;
    }
}
