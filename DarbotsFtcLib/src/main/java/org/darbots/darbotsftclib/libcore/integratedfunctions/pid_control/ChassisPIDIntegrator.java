package org.darbots.darbotsftclib.libcore.integratedfunctions.pid_control;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.RobotPose2D;

public class ChassisPIDIntegrator {
    public static class ChassisPIDErrorInfo{
        public RobotPose2D Error;
        public RobotPose2D Integral;
        public RobotPose2D Derivative;
        public ChassisPIDErrorInfo(){
            this.Error = new RobotPose2D(0,0,0);
            this.Integral = new RobotPose2D(0,0,0);
            this.Derivative = new RobotPose2D(0,0,0);
        }
        public ChassisPIDErrorInfo(RobotPose2D Error, RobotPose2D Integral, RobotPose2D Derivative){
            this.Error = new RobotPose2D(Error);
            this.Integral = new RobotPose2D(Integral);
            this.Derivative = new RobotPose2D(Derivative);
        }
        public ChassisPIDErrorInfo(ChassisPIDErrorInfo Info){
            this.Error = new RobotPose2D(Info.Error);
            this.Integral = new RobotPose2D(Info.Integral);
            this.Derivative = new RobotPose2D(Info.Derivative);
        }
    }
    RobotPose2D m_IntegratedError;
    RobotPose2D m_DerivedError;
    RobotPose2D m_LastError;
    ElapsedTime m_ErrorTime;
    public ChassisPIDIntegrator(){
        this.m_IntegratedError = new RobotPose2D(0,0,0);
        this.m_DerivedError = new RobotPose2D(0,0,0);
        this.m_LastError = new RobotPose2D(0,0,0);
        this.m_ErrorTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }
    public ChassisPIDIntegrator(ChassisPIDIntegrator oldCalculator){
        this.m_IntegratedError = new RobotPose2D(oldCalculator.m_IntegratedError);
        this.m_DerivedError = new RobotPose2D(0,0,0);
        this.m_LastError = new RobotPose2D(oldCalculator.m_LastError);
        this.m_ErrorTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }
    public void feedError(RobotPose2D error){
        double deltaTime = this.m_ErrorTime.seconds();
        double deltaErrorX = this.m_LastError.X - error.X, deltaErrorY = this.m_LastError.Y - error.Y, deltaErrorRotZ = this.m_LastError.getRotationZ() - error.getRotationZ();

        this.m_IntegratedError.offsetValues(error.X * deltaTime, error.Y * deltaTime, error.getRotationZ() * deltaTime);

        this.m_DerivedError.setValues(deltaErrorX / deltaTime,deltaErrorY / deltaTime,deltaErrorRotZ / deltaTime);

        this.m_LastError.setValues(error);
    }
    public void feedError(double X, double Y, double RotZ){
        double deltaTime = this.m_ErrorTime.seconds();
        double deltaErrorX = this.m_LastError.X - X, deltaErrorY = this.m_LastError.Y - Y, deltaErrorRotZ = this.m_LastError.getRotationZ() - RotZ;

        this.m_IntegratedError.offsetValues(X * deltaTime, X * deltaTime, RotZ * deltaTime);

        this.m_DerivedError.setValues(deltaErrorX / deltaTime,deltaErrorY / deltaTime,deltaErrorRotZ / deltaTime);

        this.m_LastError.setValues(X,Y,RotZ);
    }
    public void clearPreviousErrors(){
        this.m_LastError.setValues(0,0,0);
        this.m_IntegratedError.setValues(0,0,0);
        this.m_DerivedError.setValues(0,0,0);
    }
    public ChassisPIDErrorInfo getPIDErrorInfo(){
        return new ChassisPIDErrorInfo(this.m_LastError,this.m_IntegratedError,this.m_DerivedError);
    }
}
