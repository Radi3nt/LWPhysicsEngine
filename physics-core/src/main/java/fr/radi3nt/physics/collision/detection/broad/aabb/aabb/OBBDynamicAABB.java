package fr.radi3nt.physics.collision.detection.broad.aabb.aabb;

import fr.radi3nt.maths.components.advanced.quaternions.ComponentsQuaternion;
import fr.radi3nt.maths.components.advanced.quaternions.Quaternion;
import fr.radi3nt.maths.components.vectors.Vector3f;
import fr.radi3nt.maths.components.vectors.implementations.SimpleVector3f;
import fr.radi3nt.physics.collision.detection.broad.aabb.aabb.mapping.AxisMapping;
import fr.radi3nt.physics.core.TransformedObject;

import static java.lang.Math.abs;
import static java.lang.Math.max;

public class OBBDynamicAABB extends DynamicAABB {

    private final TransformedObject transformedObject;
    private final Vector3f size;

    private final Vector3f offset;
    private final Quaternion rot;

    Vector3f frozenSize = null;
    Vector3f frozenPos;

    public OBBDynamicAABB(TransformedObject transformedObject, Vector3f size, Vector3f offset, Quaternion rot) {
        this.transformedObject = transformedObject;
        this.size = size;
        this.offset = offset;
        this.rot = rot;
    }

    public OBBDynamicAABB(TransformedObject transformedObject, Vector3f size) {
        this(transformedObject, size, new SimpleVector3f(), ComponentsQuaternion.zero());
    }

    @Override
    public AxisMapping getxMapping(AxisMapping mapping) {
        calculateAxisIfMissing();
        mapping.set(frozenPos.getX()- frozenSize.getX(), frozenPos.getX()+ frozenSize.getX());
        return mapping;
    }

    @Override
    public AxisMapping getyMapping(AxisMapping mapping) {
        calculateAxisIfMissing();
        mapping.set(frozenPos.getY()- frozenSize.getY(), frozenPos.getY()+ frozenSize.getY());
        return mapping;
    }

    @Override
    public AxisMapping getzMapping(AxisMapping mapping) {
        calculateAxisIfMissing();
        mapping.set(frozenPos.getZ()- frozenSize.getZ(), frozenPos.getZ()+ frozenSize.getZ());
        return mapping;
    }

    private void calculateAxisIfMissing() {
        if (frozenSize != null) {
            return;
        }

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


        Result result = new Result(0, 0, 0);
        isolateMax(axis[0].duplicate().add(axis[1]).add(axis[2]), result);
        isolateMax(axis[0].duplicate().add(axis[1]).sub(axis[2]), result);
        isolateMax(axis[0].duplicate().sub(axis[1]).add(axis[2]), result);
        isolateMax(axis[0].duplicate().sub(axis[1]).sub(axis[2]), result);

        /*
        isolateMax(axis[0].duplicate().negate().add(axis[1]).add(axis[2]), result);
        isolateMax(axis[0].duplicate().negate().add(axis[1]).sub(axis[2]), result);
        isolateMax(axis[0].duplicate().negate().sub(axis[1]).add(axis[2]), result);

         */
        //isolateMax(axis[0].duplicate().negate().sub(axis[1]).sub(axis[2]), result);
        frozenSize = new SimpleVector3f(result.maxX, result.maxY, result.maxZ);

        frozenPos = offset.duplicate();
        transformedObject.toWorldSpace(frozenPos, frozenPos);
    }

    private static void isolateMax(Vector3f constructedPoint, Result result) {
        float maxX = Math.max(abs(constructedPoint.getX()), result.maxX);
        float maxY = Math.max(abs(constructedPoint.getY()), result.maxY);
        float maxZ = Math.max(abs(constructedPoint.getZ()), result.maxZ);
        result.maxX = maxX;
        result.maxY = maxY;
        result.maxZ = maxZ;
    }

    private static class Result {
        public float maxX;
        public float maxY;
        public float maxZ;

        public Result(float maxX, float maxY, float maxZ) {
            this.maxX = maxX;
            this.maxY = maxY;
            this.maxZ = maxZ;
        }
    }
}
