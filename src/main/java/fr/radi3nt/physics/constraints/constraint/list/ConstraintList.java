package fr.radi3nt.physics.constraints.constraint.list;

import fr.radi3nt.physics.constraints.constraint.Constraint;

import java.util.Collection;
import java.util.List;

public interface ConstraintList {

    void prepare();
    List<Constraint> getConstraints();
    void done();

}
