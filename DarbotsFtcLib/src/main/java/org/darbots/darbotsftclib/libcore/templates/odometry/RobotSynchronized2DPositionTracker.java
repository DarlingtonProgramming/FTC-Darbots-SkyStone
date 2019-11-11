package org.darbots.darbotsftclib.libcore.templates.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot2DPositionIndicator;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;

public class RobotSynchronized2DPositionTracker extends RobotBasic2DPositionTracker {
    public RobotSynchronized2DPositionTracker(Robot2DPositionIndicator initialPosition) {
        super(initialPosition);
    }

    public RobotSynchronized2DPositionTracker(Robot2DPositionIndicator initialPosition, double RobotWidth, double RobotHeight) {
        super(initialPosition, RobotWidth, RobotHeight);
    }

    public RobotSynchronized2DPositionTracker(Robot2DPositionTracker oldTracker) {
        super(oldTracker);
    }

    public Robot2DPositionIndicator getInitialPos(){
        synchronized (super.getInitialPos()){
            return new Robot2DPositionIndicator(super.getInitialPos());
        }
    }
    public void setInitialPos(Robot2DPositionIndicator initialPos){
        synchronized (super.getInitialPos()) {
            Robot2DPositionIndicator readInitialPos = super.getInitialPos();
            readInitialPos.setX(initialPos.getX());
            readInitialPos.setY(initialPos.getY());
            readInitialPos.setRotationZ(initialPos.getRotationZ());
        }
    }
    public Robot2DPositionIndicator getCurrentPosition(){
        synchronized (super.getCurrentPosition()) {
            return new Robot2DPositionIndicator(super.getCurrentPosition());
        }
    }
    public void setCurrentPosition(Robot2DPositionIndicator currentPosition){
        synchronized (super.getCurrentPosition()) {
            Robot2DPositionIndicator read2DPosition = super.getCurrentPosition();
            read2DPosition.setX(currentPosition.getX());
            read2DPosition.setY(currentPosition.getY());
            read2DPosition.setRotationZ(currentPosition.getRotationZ());
        }
    }
    public double getRobotWidth(){
        synchronized (this) {
            return super.getRobotWidth();
        }
    }
    public void setRobotWidth(double RobotWidth){
        synchronized (this) {
            super.setRobotWidth(RobotWidth);
        }
    }
    public double getRobotHeight(){
        synchronized (this) {
            return super.getRobotHeight();
        }
    }
    public void setRobotHeight(double Height){
        synchronized (this) {
            super.setRobotHeight(Height);
        }
    }
    public Robot2DPositionIndicator getRobotAxisLeftTopExtremePoint(){
        return new Robot2DPositionIndicator(-this.getRobotWidth() / 2,this.getRobotHeight()/2,0);
    }
    public Robot2DPositionIndicator getRobotAxisRightTopExtremePoint(){
        return new Robot2DPositionIndicator(this.getRobotWidth()/2,this.getRobotHeight()/2,0);
    }
    public Robot2DPositionIndicator getRobotAxisLeftBottomExtremePoint(){
        return new Robot2DPositionIndicator(-this.getRobotWidth()/2,-this.getRobotHeight()/2,0);
    }
    public Robot2DPositionIndicator getRobotAxisRightBottomExtremePoint(){
        return new Robot2DPositionIndicator(this.getRobotWidth()/2,-this.getRobotHeight()/2,0);
    }
    public void offsetPosition(Robot2DPositionIndicator offsetPosition) {
        synchronized (super.getCurrentPosition()) {
            super.offsetPosition(offsetPosition);
        }
    }
    @Override
    public void resetRelativeOffset() {
        synchronized (this.m_RelativeOffset){
            super.resetRelativeOffset();
        }
    }

    @Override
    public Robot2DPositionIndicator getRelativeOffset() {
        synchronized (this.m_RelativeOffset){
            return new Robot2DPositionIndicator(this.m_RelativeOffset);
        }
    }

    @Override
    protected void offsetRelative(Robot2DPositionIndicator offsetRobot) {
        synchronized (this.m_RelativeOffset) {
            double originalX = this.m_RelativeOffset.getX(), originalY = this.m_RelativeOffset.getY(), originalRotZ = this.m_RelativeOffset.getRotationZ();
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
            this.m_RelativeOffset.setX(newX);
            this.m_RelativeOffset.setY(newY);
            this.m_RelativeOffset.setRotationZ(newRotZ);
        }
    }
}
