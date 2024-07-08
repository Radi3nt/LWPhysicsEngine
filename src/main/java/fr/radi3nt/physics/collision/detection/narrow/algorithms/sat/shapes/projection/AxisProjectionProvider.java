package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.projection;

import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.maths.pool.ObjectPool;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.mapping.ProjectionSettable;
import fr.radi3nt.physics.core.TransformedObject;

import java.util.ArrayList;
import java.util.Collection;

import static java.lang.Math.abs;

public class AxisProjectionProvider implements SatProjectionProvider {

    private final Vector3f size;
    private final Vector3f offset;
    private final Quaternion rot;
    private final TransformedObject transformedObject;

    private Vector3f frozenPos;
    private Vector3f[] points;

    public AxisProjectionProvider(Vector3f size, Vector3f offset, Quaternion rot, TransformedObject transformedObject) {
        this.size = size;
        this.offset = offset;
        this.rot = rot;
        this.transformedObject = transformedObject;
    }

    private void compute(Vector3f size, Vector3f offset, Quaternion rot, TransformedObject transformedObject) {
        Vector3f[] axis = new Vector3f[] {
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
        transformedObject.toWorldSpace(frozenPos, frozenPos);

        points = new Vector3f[4];

        points[0] = axis[0].duplicate().add(axis[1]).add(axis[2]);
        points[1] = axis[0].duplicate().add(axis[1]).sub(axis[2]);
        points[2] = axis[0].duplicate().sub(axis[1]).add(axis[2]);
        points[3] = axis[0].duplicate().sub(axis[1]).sub(axis[2]);

        /*
        points[4] = axis[0].duplicate().negate().add(axis[1]).add(axis[2]);
        points[5] = axis[0].duplicate().negate().add(axis[1]).sub(axis[2]);
        points[6] = axis[0].duplicate().negate().sub(axis[1]).add(axis[2]);
         */

        //points[7] = axis[0].duplicate().negate().sub(axis[1]).sub(axis[2]);
    }

    @Override
    public void project(ProjectionSettable pSet, Vector3f satAxis) {
        if (frozenPos==null)
            compute(size, offset, rot, transformedObject);

        float dottedPos = frozenPos.dot(satAxis);

        float maxOnProj = 0;
        for (Vector3f point : points) {
            float dot = abs(point.dot(satAxis));
            maxOnProj = Math.max(dot, maxOnProj);
        }

        float min = dottedPos-maxOnProj;
        float max = dottedPos+maxOnProj;

        pSet.set(min, max);
    }

    @Override
    public void free(ObjectPool<Vector3f> pool) {

    }
}
