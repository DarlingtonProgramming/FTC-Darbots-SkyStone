package org.darbots.darbotsftclib.libcore.motion_planning.paths.SplinePath;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotPathIterator;

import java.util.NoSuchElementException;

public class SplinePathIterator implements RobotPathIterator {
    private double m_Resolution;
    private RobotPathIteratorStatus m_CurrentStatus;
    private SplinePath m_Path;
    private double m_TotalDistance;

    public SplinePathIterator(SplinePath path){
        this.m_Path = path;
        this.__setup();
    }

    public SplinePathIterator(SplinePathIterator oldIterator){
        this.m_Resolution = oldIterator.m_Resolution;
        this.m_CurrentStatus = new RobotPathIteratorStatus(oldIterator.m_CurrentStatus);
        this.m_Path = oldIterator.m_Path;
        this.m_TotalDistance = oldIterator.m_TotalDistance;
    }

    private void __setup(){
        RobotPoint2D currentPoint = new RobotPoint2D(0,0);
        this.m_CurrentStatus = new RobotPathIteratorStatus(0,currentPoint);
        this.m_TotalDistance = this.m_Path.getTotalDistance();
        this.m_Resolution = this.m_Path.m_PathIntegrationResolution;
    }

    @Override
    public RobotPathIteratorStatus forward(double deltaDistance) throws NoSuchElementException {
        if(deltaDistance == 0){
            return this.current();
        }
        double targetDistance = this.m_CurrentStatus.currentDistance + deltaDistance;
        if(targetDistance < 0 || targetDistance > this.m_TotalDistance){
            throw new NoSuchElementException("Distance Out Of Bound");
        }
        boolean reversedSpline = this.m_Path.m_Spline.getMinX() != 0;
        double distanceCounter = this.m_CurrentStatus.currentDistance;
        boolean xSpline = this.m_Path.m_SplineType == SplinePath.SplineType.X_BASED_SPLINE;
        double xCounter;
        double yCounter;

        if(xSpline){
            xCounter = this.m_CurrentStatus.currentPoint.X;
            yCounter = this.m_CurrentStatus.currentPoint.Y;
        }else{
            xCounter = this.m_CurrentStatus.currentPoint.Y;
            yCounter = this.m_CurrentStatus.currentPoint.X;
        }

        {
            double currentXRemainder = xCounter % this.m_Resolution;
            if(currentXRemainder != 0){
                double newX = xCounter - currentXRemainder;
                double newY = this.m_Path.m_Spline.interpolate(xCounter);
                distanceCounter -= Math.sqrt(Math.pow(newY - yCounter,2) + Math.pow(newX - xCounter,2));
                xCounter = newX;
                yCounter = newY;
            }
        }
        boolean forwardDistance = distanceCounter < targetDistance;
        double deltaXEveryTime = reversedSpline ? -this.m_Resolution : this.m_Resolution;
        if(!forwardDistance){
            deltaXEveryTime = -deltaXEveryTime;
        }
        double extremeX;
        double targetX, targetY;
        double deltaX, deltaY;
        double targetDeltaDistance;
        if(forwardDistance){
            extremeX = reversedSpline ? this.m_Path.m_Spline.getMinX() : this.m_Path.m_Spline.getMaxX();
            while(true){
                targetX = xCounter + deltaXEveryTime;
                if(reversedSpline){
                    if(targetX < extremeX){
                        targetX = extremeX;
                    }
                }else{
                    if(targetX > extremeX){
                        targetX = extremeX;
                    }
                }
                targetY = this.m_Path.m_Spline.interpolate(targetX);
                deltaX = targetX - xCounter;
                deltaY = targetY - yCounter;
                targetDeltaDistance = Math.sqrt(Math.pow(deltaX,2) + Math.pow(deltaY,2));
                if(targetDeltaDistance + distanceCounter == targetDistance){
                    return returnStatus(targetX,targetY,targetDistance);
                }else if(targetDeltaDistance + distanceCounter > targetDistance){
                    double remainingHypo = targetDistance - distanceCounter;
                    double factor = remainingHypo / targetDeltaDistance;
                    double factoredDeltaX = deltaX * factor;
                    double factoredDeltaY = deltaY * factor;
                    return returnStatus(xCounter + factoredDeltaX,yCounter + factoredDeltaY,targetDeltaDistance);
                }
                xCounter = targetX;
                yCounter = targetY;
                distanceCounter += targetDeltaDistance;
                if(targetX == extremeX){
                    throw new NoSuchElementException("Iterated through the whole spline, but did not find the point");
                }
            }
        }else{ //!forwardDistance
            extremeX = reversedSpline ? this.m_Path.m_Spline.getMaxX() : this.m_Path.m_Spline.getMinX();
            while(true){
                targetX = xCounter + deltaXEveryTime;
                if(reversedSpline){
                    if(targetX > extremeX){
                        targetX = extremeX;
                    }
                }else{
                    if(targetX < extremeX){
                        targetX = extremeX;
                    }
                }
                targetY = this.m_Path.m_Spline.interpolate(targetX);
                deltaX = targetX - xCounter;
                deltaY = targetY - yCounter;
                targetDeltaDistance = -Math.sqrt(Math.pow(deltaX,2) + Math.pow(deltaY,2));
                if(targetDeltaDistance + distanceCounter == targetDistance){
                    return returnStatus(targetX,targetY,targetDistance);
                }else if(targetDeltaDistance + distanceCounter < targetDistance){
                    double remainingHypo = targetDistance - distanceCounter;
                    double factor = remainingHypo / targetDeltaDistance;
                    double factoredDeltaX = deltaX * factor;
                    double factoredDeltaY = deltaY * factor;
                    return returnStatus(xCounter + factoredDeltaX,yCounter + factoredDeltaY,targetDeltaDistance);
                }
                xCounter = targetX;
                yCounter = targetY;
                distanceCounter += targetDeltaDistance;
                if(targetX == extremeX){
                    throw new NoSuchElementException("Iterated through the whole spline, but did not find the point");
                }
            }
        }
    }

    private RobotPathIteratorStatus returnStatus(double xCounter, double yCounter, double distanceCounter){
        boolean xSpline = this.m_Path.m_SplineType == SplinePath.SplineType.X_BASED_SPLINE;
        if(xSpline){
            this.m_CurrentStatus.currentPoint.X = xCounter;
            this.m_CurrentStatus.currentPoint.Y = yCounter;
        }else{
            this.m_CurrentStatus.currentPoint.X = yCounter;
            this.m_CurrentStatus.currentPoint.Y = xCounter;
        }
        this.m_CurrentStatus.currentDistance = distanceCounter;
        return this.m_CurrentStatus;
    }

    @Override
    public RobotPathIteratorStatus backward(double deltaBackwardDistance) throws NoSuchElementException {
        return forward(-deltaBackwardDistance);
    }

    @Override
    public RobotPathIteratorStatus current() {
        return this.m_CurrentStatus;
    }

    @Override
    public double getCurrentDistance() {
        return this.m_CurrentStatus.currentDistance;
    }

    @Override
    public double getTotalDistance() {
        return this.m_TotalDistance;
    }
}
