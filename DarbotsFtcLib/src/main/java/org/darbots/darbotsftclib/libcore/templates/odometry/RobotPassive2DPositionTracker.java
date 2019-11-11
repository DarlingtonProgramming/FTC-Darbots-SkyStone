package org.darbots.darbotsftclib.libcore.templates.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot2DPositionIndicator;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;

public abstract class RobotPassive2DPositionTracker implements Robot2DPositionTracker {
    private double m_InitialPos_X, m_InitialPos_Y, m_InitialPos_RotZ;
    private volatile double m_CurrentPos_X, m_CurrentPos_Y, m_CurrentPos_RotZ;
    private volatile double m_RobotWidth;
    private volatile double m_RobotHeight;
    protected Robot2DPositionIndicator m_RelativeOffset;

    public RobotPassive2DPositionTracker(Robot2DPositionIndicator initialPosition){
        this.m_InitialPos_X = initialPosition.getX();
        this.m_InitialPos_Y = initialPosition.getY();
        this.m_InitialPos_RotZ = initialPosition.getRotationZ();
        this.m_CurrentPos_X = m_InitialPos_X;
        this.m_CurrentPos_Y = m_InitialPos_Y;
        this.m_CurrentPos_RotZ = m_InitialPos_RotZ;
    }
    public RobotPassive2DPositionTracker(Robot2DPositionIndicator initialPosition, double RobotWidth, double RobotHeight){
        this.m_InitialPos_X = initialPosition.getX();
        this.m_InitialPos_Y = initialPosition.getY();
        this.m_InitialPos_RotZ = initialPosition.getRotationZ();
        this.m_CurrentPos_X = m_InitialPos_X;
        this.m_CurrentPos_Y = m_InitialPos_Y;
        this.m_CurrentPos_RotZ = m_InitialPos_RotZ;
        this.m_RobotWidth = RobotWidth;
        this.m_RobotHeight = RobotHeight;
    }
    public RobotPassive2DPositionTracker(Robot2DPositionTracker oldTracker){
        this.m_InitialPos_X = oldTracker.getInitialPos().getX();
        this.m_InitialPos_Y = oldTracker.getInitialPos().getY();
        this.m_InitialPos_RotZ = oldTracker.getInitialPos().getRotationZ();
        this.m_CurrentPos_X = oldTracker.getCurrentPosition().getX();
        this.m_CurrentPos_Y = oldTracker.getCurrentPosition().getY();;
        this.m_CurrentPos_RotZ = oldTracker.getCurrentPosition().getRotationZ();
        this.m_RobotWidth = oldTracker.getRobotWidth();
        this.m_RobotHeight = oldTracker.getRobotHeight();
    }

    public Robot2DPositionIndicator getInitialPos(){
        return new Robot2DPositionIndicator(this.m_InitialPos_X,this.m_InitialPos_Y,this.m_InitialPos_RotZ);
    }
    public void setInitialPos(Robot2DPositionIndicator initialPos){
        this.m_InitialPos_X = initialPos.getX();
        this.m_InitialPos_Y = initialPos.getY();
        this.m_InitialPos_RotZ = initialPos.getRotationZ();
    }
    public Robot2DPositionIndicator getCurrentPosition(){
        return new Robot2DPositionIndicator(this.m_CurrentPos_X,this.m_CurrentPos_Y,this.m_CurrentPos_RotZ);
    }
    protected void setCurrentPosition(Robot2DPositionIndicator currentPosition){
        this.m_CurrentPos_X = currentPosition.getX();
        this.m_CurrentPos_Y = currentPosition.getY();
        this.m_CurrentPos_RotZ = currentPosition.getRotationZ();
    }
    public double getRobotWidth(){
        return this.m_RobotWidth;
    }
    public void setRobotWidth(double RobotWidth){
        this.m_RobotWidth = RobotWidth;
    }
    public double getRobotHeight(){
        return this.m_RobotHeight;
    }
    public void setRobotHeight(double Height){
        this.m_RobotHeight = Height;
    }
    public Robot2DPositionIndicator getRobotAxisLeftTopExtremePoint(){
        return new Robot2DPositionIndicator(-m_RobotWidth / 2,m_RobotHeight/2,0);
    }
    public Robot2DPositionIndicator getRobotAxisRightTopExtremePoint(){
        return new Robot2DPositionIndicator(m_RobotWidth/2,m_RobotHeight/2,0);
    }
    public Robot2DPositionIndicator getRobotAxisLeftBottomExtremePoint(){
        return new Robot2DPositionIndicator(-m_RobotWidth/2,-m_RobotHeight/2,0);
    }
    public Robot2DPositionIndicator getRobotAxisRightBottomExtremePoint(){
        return new Robot2DPositionIndicator(m_RobotWidth/2,-m_RobotHeight/2,0);
    }
    public Robot2DPositionIndicator fieldAxisFromRobotAxis(Robot2DPositionIndicator RobotAxisPoint){
        return XYPlaneCalculations.getAbsolutePosition(this.getCurrentPosition(),RobotAxisPoint);
    }
    public Robot2DPositionIndicator robotAxisFromFieldAxis(Robot2DPositionIndicator FieldAxisPoint){
        return XYPlaneCalculations.getRelativePosition(this.getCurrentPosition(),FieldAxisPoint);
    }
    protected void offsetPosition(Robot2DPositionIndicator offsetPosition) {
        if(offsetPosition.getX() != 0)
            this.m_CurrentPos_X += offsetPosition.getX();
        if(offsetPosition.getY() != 0)
            this.m_CurrentPos_Y += offsetPosition.getY();
        if(offsetPosition.getRotationZ() != 0)
            this.m_CurrentPos_RotZ += offsetPosition.getRotationZ();
    }

    protected void offsetRelative(Robot2DPositionIndicator offsetRobot){
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

    @Override
    public Robot2DPositionIndicator getRelativeOffset() {
        synchronized (this.m_RelativeOffset) {
            return new Robot2DPositionIndicator(m_RelativeOffset);
        }
    }

    @Override
    public void resetRelativeOffset() {
        synchronized (this.m_RelativeOffset) {
            this.m_RelativeOffset.setX(0);
            this.m_RelativeOffset.setY(0);
            this.m_RelativeOffset.setRotationZ(0);
        }
    }


}
