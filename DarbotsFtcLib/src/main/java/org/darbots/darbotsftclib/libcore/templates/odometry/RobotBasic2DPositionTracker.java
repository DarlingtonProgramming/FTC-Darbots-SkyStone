package org.darbots.darbotsftclib.libcore.templates.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;

public abstract class RobotBasic2DPositionTracker implements Robot2DPositionTracker {
    private RobotPose2D m_InitialPos;
    private RobotPose2D m_CurrentPos;
    private RobotPose2D m_RelativeOffset;
    private RobotPose2D m_VelocityVector;

    public RobotBasic2DPositionTracker(RobotPose2D initialPosition){
        this.m_InitialPos = new RobotPose2D(initialPosition);
        this.m_CurrentPos = new RobotPose2D(initialPosition);
        this.m_RelativeOffset = new RobotPose2D(0,0,0);
        this.m_VelocityVector = new RobotPose2D(0,0,0);
    }

    public RobotBasic2DPositionTracker(Robot2DPositionTracker oldTracker){
        this.m_InitialPos = new RobotPose2D(oldTracker.getInitialPos());
        this.m_CurrentPos = new RobotPose2D(oldTracker.getCurrentPosition());
        this.m_RelativeOffset = new RobotPose2D(oldTracker.getRelativeOffset());
        this.m_VelocityVector = new RobotPose2D(oldTracker.getCurrentVelocityVector());
    }

    @Override
    public RobotPose2D getInitialPos(){
        return this.m_InitialPos;
    }

    public void setInitialPos(RobotPose2D initialPos){
        this.m_InitialPos.setX(initialPos.getX());
        this.m_InitialPos.setY(initialPos.getY());
        this.m_InitialPos.setRotationZ(initialPos.getRotationZ());
    }

    @Override
    public RobotPose2D getCurrentPosition(){
        return this.m_CurrentPos;
    }

    public void setCurrentPosition(RobotPose2D currentPosition){
        this.m_CurrentPos.setX(currentPosition.getX());
        this.m_CurrentPos.setY(currentPosition.getY());
        this.m_CurrentPos.setRotationZ(currentPosition.getRotationZ());
    }

    @Override
    public RobotPose2D fieldAxisFromRobotAxis(RobotPose2D RobotAxisPoint){
        return XYPlaneCalculations.getAbsolutePosition(this.getCurrentPosition(),RobotAxisPoint);
    }
    @Override
    public RobotPose2D robotAxisFromFieldAxis(RobotPose2D FieldAxisPoint){
        return XYPlaneCalculations.getRelativePosition(this.getCurrentPosition(),FieldAxisPoint);
    }

    @Override
    public void resetRelativeOffset() {
        this.m_RelativeOffset.setX(0);
        this.m_RelativeOffset.setY(0);
        this.m_RelativeOffset.setRotationZ(0);
    }

    @Override
    public RobotPose2D getRelativeOffset() {
        return m_RelativeOffset;
    }

    protected void offsetPosition(RobotPose2D offsetPosition) {
        RobotPose2D currentPosition = this.m_CurrentPos;
        if(offsetPosition.getX() != 0)
            currentPosition.setX(currentPosition.getX() + offsetPosition.getX());
        if(offsetPosition.getY() != 0)
            currentPosition.setY(currentPosition.getY() + offsetPosition.getY());
        if(offsetPosition.getRotationZ() != 0)
            currentPosition.setRotationZ(currentPosition.getRotationZ() + offsetPosition.getRotationZ());
    }

    protected void offsetRelative(RobotPose2D offsetRobot){
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

    @Override
    public RobotPose2D getCurrentVelocityVector(){
        return this.m_VelocityVector;
    }

    public void setCurrentVelocityVector(RobotPose2D velocityVector){
        this.m_VelocityVector.setX(velocityVector.getX());
        this.m_VelocityVector.setY(velocityVector.getY());
        this.m_VelocityVector.setRotationZ(velocityVector.getRotationZ());
    }
}
