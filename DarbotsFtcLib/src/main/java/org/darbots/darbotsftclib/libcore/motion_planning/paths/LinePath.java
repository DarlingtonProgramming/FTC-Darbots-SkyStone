package org.darbots.darbotsftclib.libcore.motion_planning.paths;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.DarbotsDerivative;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotPath;

public class LinePath implements RobotPath {
    private double targetX, targetY;

    @Override
    public double getTotalDistance() {
        return Math.sqrt(Math.pow(targetX,2) + Math.pow(targetY,2));
    }

    @Override
    public double getIndependentVariableExtreme() {
        return this.getIndependentVariable() == independentVariable.X ? targetX : targetY;
    }

    @Override
    public independentVariable getIndependentVariable() {
        if(this.targetX > targetY){
            return independentVariable.X;
        }else{ //targetX <= targetY
            return independentVariable.Y;
        }
    }

    @Override
    public RobotPoint2D getPointAtDistance(double distance) {
        double independentVal = Math.abs(distance) / this.getTotalDistance() * this.getIndependentVariableExtreme();
        double x=0, y=0;
        if(this.getIndependentVariable() == independentVariable.X){
            x = independentVal;
            y = targetY * (x / targetX);
        }else{
            y = independentVal;
            x = targetX * (y / targetY);
        }
        return new RobotPoint2D(x,y);
    }

    @Override
    public RobotPoint2D getPointAt(double independentVal) {
        double x=0, y=0;
        if(this.getIndependentVariable() == independentVariable.X){
            x = independentVal;
            y = targetY * (x / targetX);
        }else{
            y = independentVal;
            x = targetX * (y / targetY);
        }
        return new RobotPoint2D(x,y);
    }

    @Override
    public DarbotsDerivative getDerivativeYOverXAt(double independentVal) {
        return new DarbotsDerivative(this.targetX,this.targetY);
    }

    @Override
    public DarbotsDerivative getDerivativeYOverXAtDistance(double distance) {
        return new DarbotsDerivative(this.targetX,this.targetY);
    }

    @Override
    public DarbotsDerivative getDerivativeYOverXBetween(double startIndependentVal, double endIndependentVal) {
        return new DarbotsDerivative(this.targetX,this.targetY);
    }

    @Override
    public DarbotsDerivative getDerivativeYOverXBetweenDistance(double startDistance, double endDistance) {
        return new DarbotsDerivative(this.targetX,this.targetY);
    }
}
