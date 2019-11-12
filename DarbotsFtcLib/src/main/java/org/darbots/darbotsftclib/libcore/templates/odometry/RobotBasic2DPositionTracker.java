package org.darbots.darbotsftclib.libcore.templates.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot2DPositionIndicator;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;

public abstract class RobotBasic2DPositionTracker implements Robot2DPositionTracker {
    private Robot2DPositionIndicator m_InitialPos;
    private Robot2DPositionIndicator m_CurrentPos;
    protected Robot2DPositionIndicator m_RelativeOffset;

    public RobotBasic2DPositionTracker(Robot2DPositionIndicator initialPosition){
        this.m_InitialPos = new Robot2DPositionIndicator(initialPosition);
        this.m_CurrentPos = new Robot2DPositionIndicator(initialPosition);
    }

    public RobotBasic2DPositionTracker(Robot2DPositionTracker oldTracker){
        this.m_InitialPos = new Robot2DPositionIndicator(oldTracker.getInitialPos());
        this.m_CurrentPos = new Robot2DPositionIndicator(oldTracker.getCurrentPosition());
    }

    @Override
    public Robot2DPositionIndicator getInitialPos(){
        return this.m_InitialPos;
    }

    public void setInitialPos(Robot2DPositionIndicator initialPos){
        this.m_InitialPos.setX(initialPos.getX());
        this.m_InitialPos.setY(initialPos.getY());
        this.m_InitialPos.setRotationZ(initialPos.getRotationZ());
    }

    @Override
    public Robot2DPositionIndicator getCurrentPosition(){
        return this.m_CurrentPos;
    }

    public void setCurrentPosition(Robot2DPositionIndicator currentPosition){
        this.m_CurrentPos.setX(currentPosition.getX());
        this.m_CurrentPos.setY(currentPosition.getY());
        this.m_CurrentPos.setRotationZ(currentPosition.getRotationZ());
    }

    @Override
    public Robot2DPositionIndicator fieldAxisFromRobotAxis(Robot2DPositionIndicator RobotAxisPoint){
        return XYPlaneCalculations.getAbsolutePosition(this.getCurrentPosition(),RobotAxisPoint);
    }
    @Override
    public Robot2DPositionIndicator robotAxisFromFieldAxis(Robot2DPositionIndicator FieldAxisPoint){
        return XYPlaneCalculations.getRelativePosition(this.getCurrentPosition(),FieldAxisPoint);
    }

    @Override
    public void resetRelativeOffset() {
        this.m_RelativeOffset.setX(0);
        this.m_RelativeOffset.setY(0);
        this.m_RelativeOffset.setRotationZ(0);
    }

    @Override
    public Robot2DPositionIndicator getRelativeOffset() {
        return m_RelativeOffset;
    }

    protected void offsetPosition(Robot2DPositionIndicator offsetPosition) {
        Robot2DPositionIndicator currentPosition = this.m_CurrentPos;
        if(offsetPosition.getX() != 0)
            currentPosition.setX(currentPosition.getX() + offsetPosition.getX());
        if(offsetPosition.getY() != 0)
            currentPosition.setY(currentPosition.getY() + offsetPosition.getY());
        if(offsetPosition.getRotationZ() != 0)
            currentPosition.setRotationZ(currentPosition.getRotationZ() + offsetPosition.getRotationZ());
    }

    protected void offsetRelative(Robot2DPositionIndicator offsetRobot){
        double originalX = this.m_RelativeOffset.getX(), originalY = this.m_RelativeOffset.getY(), originalRotZ = this.m_RelativeOffset.getRotationZ();
        double newX=originalX, newY = originalY, newRotZ = originalRotZ;

        if(offsetRobot.getRotationZ() != 0){
            double[] originalXY = {originalX,originalY};
            double[] origin = {0,0};
            double[] newXY = XYPlaneCalculations.rotatePointAroundFixedPoint_Deg(originalXY,origin,-offsetRobot.getRotationZ());
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
