package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.projection;

import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.maths.pool.ObjectPool;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.mapping.ProjectionSettable;
import fr.radi3nt.physics.core.TransformedObject;

import static java.lang.Math.abs;

public class NewAxisProjectionProvider implements SatProjectionProvider {

    private final Vector3f frozenPos;
    private final Vector3f[] transformedPoints;

    public NewAxisProjectionProvider(Vector3f size, Vector3f offset, Quaternion rot, TransformedObject transformedObject) {
        Vector3f[] axis = new Vector3f[]{
                new SimpleVector3f(1, 0, 0),
                new SimpleVector3f(0, 1, 0),
                new SimpleVector3f(0, 0, 1),
        };

        for (Vector3f axi : axis) {
            axi.mul(size);
            rot.transform(axi);
            transformedObject.getRotation().transform(axi);
        }

        frozenPos = offset.duplicate();
        transformedObject.getRotation().transform(frozenPos);
        frozenPos.add(transformedObject.getPosition());

        transformedPoints = new Vector3f[axis.length*2];
        for (int i = 0; i < axis.length; i++) {
            Vector3f otherAxis = axis[(i + 1) % (axis.length)];
            Vector3f constructedPoint = axis[i].duplicate().add(otherAxis);
            Vector3f constructedPoint2 = axis[i].duplicate().sub(otherAxis);
            transformedPoints[i*2] = constructedPoint;
            transformedPoints[i*2+1] = constructedPoint2;
        }
    }

    @Override
    public void project(ProjectionSettable pSet, Vector3f satAxis) {
        float dottedPos = frozenPos.dot(satAxis);

        float maxOnProj = 0;
        for (Vector3f transformedPoint : transformedPoints) {
            maxOnProj = Math.max(abs(transformedPoint.dot(satAxis)), maxOnProj);
        }

        float min = dottedPos-maxOnProj;
        float max = dottedPos+maxOnProj;

        pSet.set(min, max);
    }

    @Override
    public void free(ObjectPool<Vector3f> pool) {

    }
}
