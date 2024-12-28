package fr.radi3nt.physics.core;

import fr.radi3nt.maths.components.advanced.quaternions.ComponentsQuaternion;
import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;

public interface TransformedObject {

    TransformedObject ZERO = new TransformedObject() {

        private final Vector3f zeroPos = new SimpleVector3f();
        private final Quaternion zeroRot = ComponentsQuaternion.zero();

        @Override
        public Vector3f getPosition() {
            return zeroPos.duplicate();
        }

        @Override
        public Quaternion getRotation() {
            return zeroRot.duplicate();
        }
    };

    Vector3f getPosition();
    Quaternion getRotation();

    default Vector3f toLocalSpace(Vector3f world) {
        Vector3f localPos = world.duplicate().sub(getPosition());
        Quaternion inverseRot = getRotation().duplicate();
        inverseRot.inverse();
        inverseRot.transform(localPos);
        return localPos;
    }

    default Vector3f toWorldSpace(Vector3f local) {
        Vector3f worldPos = local.duplicate();
        getRotation().transform(worldPos);
        worldPos.add(getPosition());
        return worldPos;
    }

    default Vector3f toWorldSpace(Vector3f local, Vector3f result) {
        result.copy(local);
        getRotation().transform(result);
        result.add(getPosition());
        return result;
    }

}
