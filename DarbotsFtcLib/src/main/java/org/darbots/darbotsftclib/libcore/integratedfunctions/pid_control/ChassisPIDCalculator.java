package org.darbots.darbotsftclib.libcore.integratedfunctions.pid_control;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.RobotPose2D;

public class ChassisPIDCalculator {
    public PIDCoefficients xPIDCoefficients;
    public PIDCoefficients yPIDCoefficients;
    public PIDCoefficients rotZPIDCoefficients;
    private ChassisPIDIntegrator m_PIDCalculator;
    public ChassisPIDCalculator(PIDCoefficients xPIDCoefficients, PIDCoefficients yPIDCoefficients, PIDCoefficients rotZPIDCoefficients){
        this.xPIDCoefficients = xPIDCoefficients;
        this.yPIDCoefficients = yPIDCoefficients;
        this.rotZPIDCoefficients = rotZPIDCoefficients;
        this.m_PIDCalculator = new ChassisPIDIntegrator();
    }
    public ChassisPIDCalculator(ChassisPIDCalculator oldPIDController){
        this.xPIDCoefficients = oldPIDController.xPIDCoefficients;
        this.yPIDCoefficients = oldPIDController.yPIDCoefficients;
        this.rotZPIDCoefficients = oldPIDController.rotZPIDCoefficients;
        this.m_PIDCalculator = new ChassisPIDIntegrator(oldPIDController.m_PIDCalculator);
    }
    public void clearPreviousErrors(){
        this.m_PIDCalculator.clearPreviousErrors();
    }
    public ChassisPIDIntegrator getPIDCalculator(){
        return this.m_PIDCalculator;
    }
    public RobotPose2D getPIDPower(){
        ChassisPIDIntegrator.ChassisPIDErrorInfo errorInfo = this.m_PIDCalculator.getPIDErrorInfo();
        double XPower = this.xPIDCoefficients.Kp * errorInfo.Error.getX() + this.xPIDCoefficients.Ki * errorInfo.Integral.getX() + this.xPIDCoefficients.Kd * errorInfo.Derivative.getX();
        double YPower = this.yPIDCoefficients.Kp * errorInfo.Error.getY() + this.yPIDCoefficients.Ki * errorInfo.Integral.getY() + this.yPIDCoefficients.Kd * errorInfo.Derivative.getY();
        double RotZPower = this.rotZPIDCoefficients.Kp * errorInfo.Error.getRotationZ() + this.rotZPIDCoefficients.Ki * errorInfo.Integral.getRotationZ() + this.rotZPIDCoefficients.Kd * errorInfo.Derivative.getRotationZ();
        return new RobotPose2D(XPower,YPower,RotZPower);
    }
    public RobotPose2D getPIPower(){
        ChassisPIDIntegrator.ChassisPIDErrorInfo errorInfo = this.m_PIDCalculator.getPIDErrorInfo();
        double XPower = this.xPIDCoefficients.Kp * errorInfo.Error.getX() + this.xPIDCoefficients.Ki * errorInfo.Integral.getX();
        double YPower = this.yPIDCoefficients.Kp * errorInfo.Error.getY() + this.yPIDCoefficients.Ki * errorInfo.Integral.getY();
        double RotZPower = this.rotZPIDCoefficients.Kp * errorInfo.Error.getRotationZ() + this.rotZPIDCoefficients.Ki * errorInfo.Integral.getRotationZ();
        return new RobotPose2D(XPower,YPower,RotZPower);
    }
    public RobotPose2D getPDPower(){
        ChassisPIDIntegrator.ChassisPIDErrorInfo errorInfo = this.m_PIDCalculator.getPIDErrorInfo();
        double XPower = this.xPIDCoefficients.Kp * errorInfo.Error.getX() + this.xPIDCoefficients.Kd * errorInfo.Derivative.getX();
        double YPower = this.yPIDCoefficients.Kp * errorInfo.Error.getY() + this.yPIDCoefficients.Kd * errorInfo.Derivative.getY();
        double RotZPower = this.rotZPIDCoefficients.Kp * errorInfo.Error.getRotationZ() + this.rotZPIDCoefficients.Kd * errorInfo.Derivative.getRotationZ();
        return new RobotPose2D(XPower,YPower,RotZPower);
    }
    public RobotPose2D getPPower(){
        ChassisPIDIntegrator.ChassisPIDErrorInfo errorInfo = this.m_PIDCalculator.getPIDErrorInfo();
        double XPower = this.xPIDCoefficients.Kp * errorInfo.Error.getX();
        double YPower = this.yPIDCoefficients.Kp * errorInfo.Error.getY();
        double RotZPower = this.rotZPIDCoefficients.Kp * errorInfo.Error.getRotationZ();
        return new RobotPose2D(XPower,YPower,RotZPower);
    }
    public RobotPose2D getIPower(){
        ChassisPIDIntegrator.ChassisPIDErrorInfo errorInfo = this.m_PIDCalculator.getPIDErrorInfo();
        double XPower = this.xPIDCoefficients.Ki * errorInfo.Integral.getX();
        double YPower = this.yPIDCoefficients.Ki * errorInfo.Integral.getY();
        double RotZPower = this.rotZPIDCoefficients.Ki * errorInfo.Integral.getRotationZ();
        return new RobotPose2D(XPower,YPower,RotZPower);
    }
    public RobotPose2D getDPower(){
        ChassisPIDIntegrator.ChassisPIDErrorInfo errorInfo = this.m_PIDCalculator.getPIDErrorInfo();
        double XPower = this.xPIDCoefficients.Kd * errorInfo.Derivative.getX();
        double YPower = this.yPIDCoefficients.Kd * errorInfo.Derivative.getY();
        double RotZPower = this.rotZPIDCoefficients.Kd * errorInfo.Derivative.getRotationZ();
        return new RobotPose2D(XPower,YPower,RotZPower);
    }
    public void feedError(RobotPose2D error){
        this.m_PIDCalculator.feedError(error);
    }
    public void feedError(double X, double Y, double RotZ){
        this.m_PIDCalculator.feedError(X,Y,RotZ);
    }
}
