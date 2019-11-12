package org.darbots.darbotsftclib.libcore.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot2DPositionIndicator;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.templates.odometry.Robot2DPositionTracker;
import org.darbots.darbotsftclib.libcore.templates.odometry.RobotBasic2DPositionTracker;

public class Robot2DPositionSoftwareTracker extends RobotBasic2DPositionTracker {

    public Robot2DPositionSoftwareTracker(Robot2DPositionIndicator initialPosition) {
        super(initialPosition);
    }

    public Robot2DPositionSoftwareTracker(Robot2DPositionSoftwareTracker oldTracker) {
        super(oldTracker);
    }


    public void drive_MoveThroughRobotAngle(double angleInDeg, double distance) {
        double fieldAng = angleInDeg + this.getCurrentPosition().getRotationZ();
        double angleInRad = Math.toRadians(angleInDeg);
        double fieldAngleInRad = Math.toRadians(fieldAng);
        double xMoved = Math.cos(fieldAngleInRad) * distance, yMoved = Math.sin(fieldAngleInRad) * distance;
        this.offsetPosition(new Robot2DPositionIndicator(xMoved, yMoved, 0));
        double robotXMoved = Math.cos(angleInRad) * distance, robotYMoved = Math.sin(angleInRad) * distance;
        this.offsetRelative(new Robot2DPositionIndicator(robotXMoved, robotYMoved, 0));
    }
    public void drive_MoveThroughRobotAxisOffset(Robot2DPositionIndicator robotAxisValues) {
        Robot2DPositionIndicator tempField = this.fieldAxisFromRobotAxis(robotAxisValues);
        this.setCurrentPosition(tempField);
        this.offsetRelative(robotAxisValues);
    }
    public void drive_RotateAroundFieldPoint(Robot2DPositionIndicator fieldPointAndRotation) {
        Robot2DPositionIndicator currentPos = this.getCurrentPosition();
        if (fieldPointAndRotation.getX() == currentPos.getX() && fieldPointAndRotation.getY() == currentPos.getY()) {
            Robot2DPositionIndicator deltaVal = new Robot2DPositionIndicator(0,0,fieldPointAndRotation.getRotationZ());
            this.offsetPosition(deltaVal);
            this.offsetRelative(deltaVal);
        } else {
            double[] point = {fieldPointAndRotation.getX(), fieldPointAndRotation.getY()};
            double[] currentPosArr = {currentPos.getX(), currentPos.getY()};
            double[] newRobotPosition = XYPlaneCalculations.rotatePointAroundFixedPoint_Deg(point, currentPosArr, fieldPointAndRotation.getRotationZ());
            Robot2DPositionIndicator newPosition = new Robot2DPositionIndicator(newRobotPosition[0], newRobotPosition[1], currentPos.getRotationZ() + fieldPointAndRotation.getRotationZ());
            this.setCurrentPosition(newPosition);
            this.offsetRelative(XYPlaneCalculations.getRelativePosition(currentPos,newPosition));
        }
    }
    public void drive_RotateAroundRobotAxisPoint(Robot2DPositionIndicator robotPointAndRotation) {
        if(robotPointAndRotation.getX() == 0 && robotPointAndRotation.getY() == 0){
            if(robotPointAndRotation.getRotationZ() != 0) {
                Robot2DPositionIndicator deltaVal = new Robot2DPositionIndicator(0,0,robotPointAndRotation.getRotationZ());
                this.offsetPosition(deltaVal);
                this.offsetRelative(deltaVal);
            }
            return;
        }
        Robot2DPositionIndicator tempFieldPointAndRotation = this.fieldAxisFromRobotAxis(robotPointAndRotation);
        tempFieldPointAndRotation.setRotationZ(robotPointAndRotation.getRotationZ());
        this.drive_RotateAroundFieldPoint(tempFieldPointAndRotation);
    }
    public void drive_RotateAroundRobotOriginWithRadius(double Radius, double DistanceCounterClockwise){
        if(DistanceCounterClockwise != 0) {
            double moveAngleRad = DistanceCounterClockwise / Radius;
            double moveAngleDeg = Math.toDegrees(moveAngleRad);
            Robot2DPositionIndicator deltaVal = new Robot2DPositionIndicator(0,0,moveAngleDeg);
            this.offsetPosition(deltaVal);
            this.offsetRelative(deltaVal);
        }
    }
}
