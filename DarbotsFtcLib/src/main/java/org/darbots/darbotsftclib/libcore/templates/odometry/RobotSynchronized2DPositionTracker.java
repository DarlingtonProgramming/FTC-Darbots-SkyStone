package org.darbots.darbotsftclib.libcore.templates.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;

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
            super.setInitialPos(initialPos);
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
            read2DPosition.X = currentPosition.X;
            read2DPosition.Y = currentPosition.Y;
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
    protected void offsetRelative(RobotPose2D offsetValues) {
        RobotPose2D readRelativePos = super.getRelativeOffset();
        synchronized (readRelativePos) {
            double originalX = readRelativePos.X, originalY = readRelativePos.Y, originalRotZ = readRelativePos.getRotationZ();
            double newX = originalX + offsetValues.X, newY = originalY + offsetValues.Y, newRotZ = originalRotZ + offsetValues.getRotationZ();
            readRelativePos.X = newX;
            readRelativePos.Y = newY;
            readRelativePos.setRotationZ(newRotZ);
        }
    }

    @Override
    protected void offsetRelative_RobotAxis(RobotPose2D offsetRobotAxis){
        RobotPose2D readRelativePos = super.getRelativeOffset();
        double[] originalRobotAxis = {offsetRobotAxis.X, offsetRobotAxis.Y};
        double[] origin = {0,0};

        synchronized (readRelativePos) {
            double[] OffsetOriginPerspectiveValues = XYPlaneCalculations.rotatePointAroundFixedPoint_Deg(originalRobotAxis, origin, readRelativePos.getRotationZ());
            double originalX = readRelativePos.X, originalY = readRelativePos.Y, originalRotZ = readRelativePos.getRotationZ();
            double newX = originalX + OffsetOriginPerspectiveValues[0], newY = originalY + OffsetOriginPerspectiveValues[1], newRotZ = originalRotZ + offsetRobotAxis.getRotationZ();
            readRelativePos.X = newX;
            readRelativePos.Y = newY;
            readRelativePos.setRotationZ(newRotZ);
        }
    }

    @Override
    public RobotVector2D getCurrentVelocityVector(){
        RobotVector2D readVelocityVector = super.getCurrentVelocityVector();
        synchronized (readVelocityVector) {
            return new RobotPose2D(readVelocityVector);
        }
    }

    public void setCurrentVelocityVector(RobotVector2D velocityVector){
        RobotVector2D readVelocityVector = super.getCurrentVelocityVector();
        synchronized (readVelocityVector){
            readVelocityVector.X = velocityVector.X;
            readVelocityVector.Y = velocityVector.Y;
            readVelocityVector.setRotationZ(velocityVector.getRotationZ());
        }
    }
}
