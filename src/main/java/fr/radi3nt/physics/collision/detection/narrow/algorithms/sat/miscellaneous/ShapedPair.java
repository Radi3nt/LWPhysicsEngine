package fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.miscellaneous;

import fr.radi3nt.physics.collision.contact.GeneratedContactPair;
import fr.radi3nt.physics.collision.detection.narrow.algorithms.sat.shapes.shape.SatProcessedShape;

public class ShapedPair {

    public final GeneratedContactPair contactPair;
    public final SatProcessedShape sA;
    public final SatProcessedShape sB;

    public ShapedPair(GeneratedContactPair contactPair, SatProcessedShape sA, SatProcessedShape sB) {
        this.contactPair = contactPair;
        this.sA = sA;
        this.sB = sB;
    }


}
