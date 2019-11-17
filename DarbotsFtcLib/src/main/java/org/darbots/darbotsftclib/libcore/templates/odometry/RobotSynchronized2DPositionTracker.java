package org.darbots.darbotsftclib.libcore.templates.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;

public abstract class RobotSynchronized2DPositionTracker extends RobotBasic2DPositionTracker {

    public RobotSynchronized2DPositionTracker(RobotPose2D initialPosition) {
        super(initialPosition);
    }

    public RobotSynchronized2DPositionTracker(Robot2DPositionTracker oldTracker) {
        super(oldTracker);
    }

    public RobotPose2D getInitialPos(){
        synchronized (super.getInitialPos()){
            return new RobotPose2D(super.getInitialPos());
        }
    }
    public void setInitialPos(RobotPose2D initialPos){
        synchronized (super.getInitialPos()) {
            RobotPose2D readInitialPos = super.getInitialPos();
            readInitialPos.setX(initialPos.getX());
            readInitialPos.setY(initialPos.getY());
            readInitialPos.setRotationZ(initialPos.getRotationZ());
        }
    }
    public RobotPose2D getCurrentPosition(){
        RobotPose2D read2DPosition = super.getCurrentPosition();
        synchronized (read2DPosition) {
            return new RobotPose2D(read2DPosition);
        }
    }
    public void setCurrentPosition(RobotPose2D currentPosition){
        RobotPose2D read2DPosition = super.getCurrentPosition();
        synchronized (read2DPosition) {
            read2DPosition.setX(currentPosition.getX());
            read2DPosition.setY(currentPosition.getY());
            read2DPosition.setRotationZ(currentPosition.getRotationZ());
        }
    }

    public void offsetPosition(RobotPose2D offsetPosition) {
        RobotPose2D read2DPosition = super.getCurrentPosition();
        synchronized (read2DPosition) {
            super.offsetPosition(offsetPosition);
        }
    }
    @Override
    public void resetRelativeOffset() {
        RobotPose2D readRelativePos = super.getRelativeOffset();
        synchronized (readRelativePos){
            super.resetRelativeOffset();
        }
    }

    @Override
    public RobotPose2D getRelativeOffset() {
        RobotPose2D readRelativePos = super.getRelativeOffset();
        synchronized (readRelativePos){
            return new RobotPose2D(readRelativePos);
        }
    }

    @Override
    protected void offsetRelative(RobotPose2D offsetRobot) {
        RobotPose2D readRelativePos = super.getRelativeOffset();
        synchronized (readRelativePos) {
            double originalX = readRelativePos.getX(), originalY = readRelativePos.getY(), originalRotZ = readRelativePos.getRotationZ();
            double newX = originalX, newY = originalY, newRotZ = originalRotZ;

            if (offsetRobot.getRotationZ() != 0) {
                double[] originalXY = {originalX, originalY};
                double[] origin = {0, 0};
                double[] newXY = XYPlaneCalculations.rotatePointAroundFixedPoint_Deg(originalXY, origin, -offsetRobot.getRotationZ());
                newX = newXY[0];
                newY = newXY[1];
                newRotZ += offsetRobot.getRotationZ();
            }
            newX += offsetRobot.getX();
            newY += offsetRobot.getY();
            readRelativePos.setX(newX);
            readRelativePos.setY(newY);
            readRelativePos.setRotationZ(newRotZ);
        }
    }

    @Override
    public RobotPose2D getCurrentVelocityVector(){
        RobotPose2D readVelocityVector = super.getCurrentVelocityVector();
        synchronized (readVelocityVector) {
            return new RobotPose2D(readVelocityVector);
        }
    }

    public void setCurrentVelocityVector(RobotPose2D velocityVector){
        RobotPose2D readVelocityVector = super.getCurrentVelocityVector();
        synchronized (readVelocityVector){
            readVelocityVector.setX(velocityVector.getX());
            readVelocityVector.setY(velocityVector.getY());
            readVelocityVector.setRotationZ(velocityVector.getRotationZ());
        }
    }
}
