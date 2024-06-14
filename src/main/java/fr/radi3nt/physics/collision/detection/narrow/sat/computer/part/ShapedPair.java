package fr.radi3nt.physics.collision.detection.narrow.sat.computer.part;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.shape.sat.shape.SatShapeObject;

public class ShapedPair {

    public final GeneratedContactPair contactPair;
    public final SatShapeObject sA;
    public final SatShapeObject sB;

    public ShapedPair(GeneratedContactPair contactPair, SatShapeObject sA, SatShapeObject sB) {
        this.contactPair = contactPair;
        this.sA = sA;
        this.sB = sB;
    }


}
