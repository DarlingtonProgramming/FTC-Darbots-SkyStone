package org.darbots.darbotsftclib.libcore.templates.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot2DPositionIndicator;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;

public class RobotSynchronized2DPositionTracker extends Robot2DPositionTracker {
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
            readInitialPos.setZ(initialPos.getZ());
            readInitialPos.setRotationY(initialPos.getRotationY());
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
            read2DPosition.setZ(currentPosition.getZ());
            read2DPosition.setRotationY(currentPosition.getRotationY());
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
}
