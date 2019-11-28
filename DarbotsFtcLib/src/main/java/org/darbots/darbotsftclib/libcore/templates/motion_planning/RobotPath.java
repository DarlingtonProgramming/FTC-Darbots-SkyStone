package org.darbots.darbotsftclib.libcore.templates.motion_planning;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.DarbotsDerivative;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;

public interface RobotPath {
    public enum independentVariable{
        X,
        Y
    }
    double getTotalDistance();
    double getIndependentVariableExtreme();
    independentVariable getIndependentVariable();
    RobotPoint2D getPointAtDistance(double distance);
    RobotPoint2D getPointAt(double independentVal);
    DarbotsDerivative getDerivativeYOverXAt(double independentVal);
    DarbotsDerivative getDerivativeYOverXAtDistance(double distance);
    DarbotsDerivative getDerivativeYOverXBetween(double startIndependentVal, double endIndependentVal);
    DarbotsDerivative getDerivativeYOverXBetweenDistance(double startDistance, double endDistance);
}
