package org.darbots.darbotsftclib.libcore.templates.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot2DPositionIndicator;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;

public class RobotSynchronized2DPositionTracker extends RobotBasic2DPositionTracker {
    public RobotSynchronized2DPositionTracker(Robot2DPositionIndicator initialPosition) {
        super(initialPosition);
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
        Robot2DPositionIndicator read2DPosition = super.getCurrentPosition();
        synchronized (read2DPosition) {
            return new Robot2DPositionIndicator(read2DPosition);
        }
    }
    public void setCurrentPosition(Robot2DPositionIndicator currentPosition){
        Robot2DPositionIndicator read2DPosition = super.getCurrentPosition();
        synchronized (read2DPosition) {
            read2DPosition.setX(currentPosition.getX());
            read2DPosition.setY(currentPosition.getY());
            read2DPosition.setRotationZ(currentPosition.getRotationZ());
        }
    }

    public void offsetPosition(Robot2DPositionIndicator offsetPosition) {
        Robot2DPositionIndicator read2DPosition = super.getCurrentPosition();
        synchronized (read2DPosition) {
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
