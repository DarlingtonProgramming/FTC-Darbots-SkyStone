package org.darbots.darbotsftclib.libcore.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.templates.odometry.RobotBasic2DPositionTracker;

public class Robot2DPositionSoftwareTracker extends RobotBasic2DPositionTracker {

    public Robot2DPositionSoftwareTracker(RobotPose2D initialPosition) {
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
        this.offsetPosition(new RobotPose2D(xMoved, yMoved, 0));
        double robotXMoved = Math.cos(angleInRad) * distance, robotYMoved = Math.sin(angleInRad) * distance;
        this.offsetRelative(new RobotPose2D(robotXMoved, robotYMoved, 0));
    }
    public void drive_MoveThroughRobotAxisOffset(RobotPose2D robotAxisValues) {
        RobotPose2D tempField = this.fieldAxisFromRobotAxis(robotAxisValues);
        this.setCurrentPosition(tempField);
        this.offsetRelative(robotAxisValues);
    }
    public void drive_RotateAroundFieldPoint(RobotPose2D fieldPointAndRotation) {
        RobotPose2D currentPos = this.getCurrentPosition();
        if (fieldPointAndRotation.getX() == currentPos.getX() && fieldPointAndRotation.getY() == currentPos.getY()) {
            RobotPose2D deltaVal = new RobotPose2D(0,0,fieldPointAndRotation.getRotationZ());
            this.offsetPosition(deltaVal);
            this.offsetRelative(deltaVal);
        } else {
            double[] point = {fieldPointAndRotation.getX(), fieldPointAndRotation.getY()};
            double[] currentPosArr = {currentPos.getX(), currentPos.getY()};
            double[] newRobotPosition = XYPlaneCalculations.rotatePointAroundFixedPoint_Deg(point, currentPosArr, fieldPointAndRotation.getRotationZ());
            RobotPose2D newPosition = new RobotPose2D(newRobotPosition[0], newRobotPosition[1], currentPos.getRotationZ() + fieldPointAndRotation.getRotationZ());
            this.setCurrentPosition(newPosition);
            this.offsetRelative(XYPlaneCalculations.getRelativePosition(currentPos,newPosition));
        }
    }
    public void drive_RotateAroundRobotAxisPoint(RobotPose2D robotPointAndRotation) {
        if(robotPointAndRotation.getX() == 0 && robotPointAndRotation.getY() == 0){
            if(robotPointAndRotation.getRotationZ() != 0) {
                RobotPose2D deltaVal = new RobotPose2D(0,0,robotPointAndRotation.getRotationZ());
                this.offsetPosition(deltaVal);
                this.offsetRelative(deltaVal);
            }
            return;
        }
        RobotPose2D tempFieldPointAndRotation = this.fieldAxisFromRobotAxis(robotPointAndRotation);
        tempFieldPointAndRotation.setRotationZ(robotPointAndRotation.getRotationZ());
        this.drive_RotateAroundFieldPoint(tempFieldPointAndRotation);
    }
    public void drive_RotateAroundRobotOriginWithRadius(double Radius, double DistanceCounterClockwise){
        if(DistanceCounterClockwise != 0) {
            double moveAngleRad = DistanceCounterClockwise / Radius;
            double moveAngleDeg = Math.toDegrees(moveAngleRad);
            RobotPose2D deltaVal = new RobotPose2D(0,0,moveAngleDeg);
            this.offsetPosition(deltaVal);
            this.offsetRelative(deltaVal);
        }
    }

    @Override
    public void stop(){
        return;
    }
}
