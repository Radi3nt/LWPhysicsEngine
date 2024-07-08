package fr.radi3nt.physics.constraints.constraint.list;

import fr.radi3nt.physics.constraints.constraint.Constraint;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class SetConstraintList implements ConstraintList {

    private final List<Constraint> permanentConstraints = new ArrayList<>();

    public SetConstraintList() {

    }

    public SetConstraintList(List<Constraint> constraints) {
        permanentConstraints.addAll(constraints);
    }


    public List<Constraint> getPermanentConstraints() {
        return permanentConstraints;
    }

    @Override
    public void prepare() {

    }

    @Override
    public List<Constraint> getConstraints() {
        return Collections.unmodifiableList(permanentConstraints);
    }

    @Override
    public void done() {

    }
}
