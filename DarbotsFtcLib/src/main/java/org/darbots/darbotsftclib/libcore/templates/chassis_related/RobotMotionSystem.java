/*
MIT License

Copyright (c) 2018 DarBots Collaborators

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

package org.darbots.darbotsftclib.libcore.templates.chassis_related;

import android.support.annotation.NonNull;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.integratedfunctions.pid_control.ChassisPIDCalculator;
import org.darbots.darbotsftclib.libcore.integratedfunctions.pid_control.PIDCalculator;
import org.darbots.darbotsftclib.libcore.integratedfunctions.pid_control.PIDCoefficients;
import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.darbots.darbotsftclib.libcore.templates.odometry.Robot2DPositionTracker;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotGyro;

import java.util.ArrayList;

public abstract class RobotMotionSystem implements RobotNonBlockingDevice {
    public final static PIDCoefficients LINEAR_X_PID_DEFAULT = new PIDCoefficients(0.4,0,0.05);
    public final static PIDCoefficients LINEAR_Y_PID_DEFAULT = new PIDCoefficients(0.4,0,0.05);
    public final static PIDCoefficients ROTATIONAL_Z_PID_DEFAULT = new PIDCoefficients(0.4,0,0.05);

    private ArrayList<RobotMotionSystemTask> m_TaskLists;
    private Robot2DPositionTracker m_PosTracker;
    private double m_LinearYMotionDistanceFactor;
    private double m_LinearXMotionDistanceFactor;
    private double m_RotationalMotionDistanceFactor;
    private boolean m_PosTrackerIsAsync;
    private PIDCoefficients m_LinearXPIDCoefficient, m_LinearYPIDCoefficient, m_RotationalPIDCoefficient;
    private RobotPose2D m_AccumulatedError;
    private ChassisPIDCalculator m_PIDCalculator;
    private RobotGyro m_Gyro;

    public RobotMotionSystem(Robot2DPositionTracker PositionTracker){
        this.m_TaskLists = new ArrayList();
        this.m_PosTracker = PositionTracker;
        this.setLinearMotionDistanceFactor(1);
        this.m_RotationalMotionDistanceFactor = 1;
        this.m_LinearXPIDCoefficient = LINEAR_X_PID_DEFAULT;
        this.m_LinearYPIDCoefficient = LINEAR_Y_PID_DEFAULT;
        this.m_RotationalPIDCoefficient = ROTATIONAL_Z_PID_DEFAULT;
        this.m_PIDCalculator = new ChassisPIDCalculator(this.m_LinearXPIDCoefficient,this.m_LinearYPIDCoefficient,this.m_RotationalPIDCoefficient);
        this.m_AccumulatedError = new RobotPose2D(0,0,0);

        if(PositionTracker != null && PositionTracker instanceof RobotNonBlockingDevice){
            this.m_PosTrackerIsAsync = true;
        }else{
            this.m_PosTrackerIsAsync = false;
        }
    }
    public RobotMotionSystem(RobotMotionSystem MotionSystem){
        this.m_TaskLists = new ArrayList();
        this.m_PosTracker = MotionSystem.m_PosTracker;
        this.m_LinearYMotionDistanceFactor = MotionSystem.m_LinearYMotionDistanceFactor;
        this.m_LinearXMotionDistanceFactor = MotionSystem.m_LinearXMotionDistanceFactor;
        this.m_RotationalMotionDistanceFactor = MotionSystem.m_RotationalMotionDistanceFactor;
        this.m_LinearXPIDCoefficient = MotionSystem.m_LinearXPIDCoefficient;
        this.m_LinearYPIDCoefficient = MotionSystem.m_LinearYPIDCoefficient;
        this.m_RotationalPIDCoefficient = MotionSystem.m_RotationalPIDCoefficient;
        this.m_PIDCalculator = new ChassisPIDCalculator(this.m_LinearXPIDCoefficient,this.m_LinearYPIDCoefficient,this.m_RotationalPIDCoefficient);
        this.m_AccumulatedError = new RobotPose2D(MotionSystem.m_AccumulatedError);

        if(MotionSystem.m_PosTracker != null && MotionSystem.m_PosTracker instanceof RobotNonBlockingDevice){
            this.m_PosTrackerIsAsync = true;
        }else{
            this.m_PosTrackerIsAsync = false;
        }
    }

    public RobotGyro getGyroValueProvider(){
        return this.m_Gyro;
    }

    public void setGyroValueProvider(RobotGyro GyroValueProvider){
        this.m_Gyro = GyroValueProvider;
    }

    public ChassisPIDCalculator getPIDCalculator(){
        return this.m_PIDCalculator;
    }

    public double getLinearYMotionDistanceFactor(){
        return this.m_LinearYMotionDistanceFactor;
    }
    public void setLinearYMotionDistanceFactor(double Factor){
        this.m_LinearYMotionDistanceFactor = Factor;
    }

    public double getLinearXMotionDistanceFactor(){
        return this.m_LinearXMotionDistanceFactor;
    }
    public void setLinearXMotionDistanceFactor(double Factor){
        this.m_LinearXMotionDistanceFactor = Factor;
    }
    public void setLinearMotionDistanceFactor(double Factor){
        this.setLinearYMotionDistanceFactor(Factor);
        this.setLinearXMotionDistanceFactor(Factor);
    }

    public double getRotationalMotionDistanceFactor(){
        return this.m_RotationalMotionDistanceFactor;
    }

    public void setRotationalMotionDistanceFactor(double Factor){
        this.m_RotationalMotionDistanceFactor = Factor;
    }

    public PIDCoefficients getLinearXPIDCoefficient(){
        return this.m_LinearXPIDCoefficient;
    }

    public void setLinearXPIDCoefficient(@NonNull PIDCoefficients LinearXPID){
        this.m_LinearXPIDCoefficient = LinearXPID;
        this.m_PIDCalculator.xPIDCoefficients = LinearXPID;
    }

    public PIDCoefficients getLinearYPIDCoefficient(){
        return this.m_LinearYPIDCoefficient;
    }

    public void setLinearYPIDCoefficient(@NonNull PIDCoefficients LinearYPID){
        this.m_LinearYPIDCoefficient = LinearYPID;
        this.m_PIDCalculator.yPIDCoefficients = LinearYPID;
    }

    public void setLinearPIDCoefficients(@NonNull PIDCoefficients LinearPIDs){
        this.m_LinearXPIDCoefficient = LinearPIDs;
        this.m_LinearYPIDCoefficient = LinearPIDs;
        this.m_PIDCalculator.xPIDCoefficients = LinearPIDs;
        this.m_PIDCalculator.yPIDCoefficients = LinearPIDs;
    }

    public PIDCoefficients getRotationalPIDCoefficient(){
        return this.m_RotationalPIDCoefficient;
    }

    public void setRotationalPIDCoefficient(@NonNull PIDCoefficients RotationalPID){
        this.m_RotationalPIDCoefficient = RotationalPID;
        this.m_PIDCalculator.rotZPIDCoefficients = RotationalPID;
    }

    public Robot2DPositionTracker getPositionTracker(){
        return this.m_PosTracker;
    }
    public void setPositionTracker(Robot2DPositionTracker PositionTracker){
        this.m_PosTracker = PositionTracker;
        if(PositionTracker != null && PositionTracker instanceof RobotNonBlockingDevice){
            this.m_PosTrackerIsAsync = true;
        }else{
            this.m_PosTrackerIsAsync = false;
        }
    }
    public void addTask(@NonNull RobotMotionSystemTask Task){
        this.m_TaskLists.add(Task);
        this.scheduleTasks();
    }

    public void replaceTask(@NonNull RobotMotionSystemTask Task){
        if(!this.m_TaskLists.isEmpty()){
             if(this.m_TaskLists.get(0).isBusy())
                 this.m_TaskLists.get(0).stopTask();
        }
        this.m_TaskLists.clear();
        this.m_TaskLists.add(Task);
        this.scheduleTasks();
    }

    public void deleteCurrentTask(){
        if(this.m_TaskLists.isEmpty()){
            this.__stopMotion();
            return;
        }
        this.m_TaskLists.get(0).stopTask();
        this.m_TaskLists.remove(0);
        this.scheduleTasks();
    }

    public ArrayList<RobotMotionSystemTask> getTaskLists(){
        return this.m_TaskLists;
    }

    public RobotMotionSystemTask getCurrentTask(){
        return this.m_TaskLists.isEmpty() ? null : this.m_TaskLists.get(0);
    }

    public void deleteAllTasks(){
        if(this.m_TaskLists.isEmpty()){
            this.__stopMotion();
            return;
        }
        RobotMotionSystemTask currentTask = this.m_TaskLists.get(0);
        this.m_TaskLists.get(0).stopTask();
        this.m_TaskLists.clear();
        this.__stopMotion();
    }

    public void __checkTasks(){
        if(this.m_TaskLists.isEmpty()){
            this.__stopMotion();
            return;
        }else if(!this.m_TaskLists.get(0).isBusy()){
            this.deleteCurrentTask();
        }
    }

    protected abstract void __stopMotion();

    protected void scheduleTasks(){
        if(!this.m_TaskLists.isEmpty()){
            if(!this.m_TaskLists.get(0).isBusy()) {
                this.m_TaskLists.get(0).setMotionSystem(this);
                this.m_TaskLists.get(0).startTask();
            }
        }else{
            this.__stopMotion();
        }
    }

    @Override
    public boolean isBusy(){
        return (!this.m_TaskLists.isEmpty());
    }

    @Override
    public void updateStatus(){
        this.__updateMotorStatus();
        if((!this.m_TaskLists.isEmpty())){
            if(this.m_TaskLists.get(0).isBusy())
                this.m_TaskLists.get(0).updateStatus();
        }
        if(this.m_PosTrackerIsAsync){
            RobotNonBlockingDevice NonBlockingTracker = (RobotNonBlockingDevice) this.m_PosTracker;
            NonBlockingTracker.updateStatus();
        }
    }

    protected abstract void __updateMotorStatus();

    @Override
    public void waitUntilFinish(){
        while(this.isBusy()){
            if(GlobalRegister.runningOpMode != null){
                if(GlobalRegister.runningOpMode.isStarted() && (!GlobalRegister.runningOpMode.opModeIsActive())){
                    return;
                }
            }
            this.updateStatus();
        }
    }

    public RobotPose2D getAccumulatedError(){
        return this.m_AccumulatedError;
    }

    public void setAccumulatedError(RobotPose2D AccumulatedError){
        this.m_AccumulatedError.X = AccumulatedError.X;
        this.m_AccumulatedError.Y = AccumulatedError.Y;
        this.m_AccumulatedError.setRotationZ(AccumulatedError.getRotationZ());
    }

    public void stop(){
        this.deleteAllTasks();
        if(this.getPositionTracker() != null){
            this.getPositionTracker().stop();
        }
    }

    public abstract RobotPose2D getTheoreticalMaximumMotionState(double WantedXSpeedInCMPerSec, double WantedYSpeedInCMPerSec, double WantedZRotSpeedInDegPerSec);
    public RobotPose2D getTheoreticalMaximumMotionState(RobotPose2D WantedVelocityVector){
        return this.getTheoreticalMaximumMotionState(WantedVelocityVector.X,WantedVelocityVector.Y,WantedVelocityVector.getRotationZ());
    }
    protected abstract void __setRobotSpeed(double XSpeedInCMPerSec, double YSpeedInCMPerSec, double ZRotSpeedInDegPerSec);
    public RobotPose2D setRobotSpeed(RobotPose2D VelocityVector){
        return this.setRobotSpeed(VelocityVector.X,VelocityVector.Y,VelocityVector.getRotationZ());
    }
    public RobotPose2D setRobotSpeed(double XSpeedInCMPerSec, double YSpeedInCMPerSec, double ZRotSpeedInDegPerSec){
        RobotPose2D biggestSpeed = this.getTheoreticalMaximumMotionState(XSpeedInCMPerSec,YSpeedInCMPerSec,ZRotSpeedInDegPerSec);
        if(Math.abs(XSpeedInCMPerSec) > Math.abs(biggestSpeed.X) || Math.abs(YSpeedInCMPerSec) > Math.abs(biggestSpeed.Y) || Math.abs(ZRotSpeedInDegPerSec) > Math.abs(biggestSpeed.getRotationZ())){
            this.__setRobotSpeed(biggestSpeed.X,biggestSpeed.Y,biggestSpeed.getRotationZ());
            return biggestSpeed;
        }
        this.__setRobotSpeed(XSpeedInCMPerSec,YSpeedInCMPerSec,ZRotSpeedInDegPerSec);
        return new RobotPose2D(XSpeedInCMPerSec,YSpeedInCMPerSec,ZRotSpeedInDegPerSec);
    }
    public abstract double[] calculateWheelAngularSpeeds(double RobotXSpeedInCMPerSec, double RobotYSpeedInCMPerSec, double RobotZRotSpeedInDegPerSec);
    public double[] calculateWheelAngularSpeeds(RobotPose2D RobotVelocity){
        return this.calculateWheelAngularSpeeds(RobotVelocity.X,RobotVelocity.Y,RobotVelocity.getRotationZ());
    }
    public abstract RobotPose2D calculateRobotSpeed(double[] wheelSpeeds);
    public double calculateMaxLinearSpeedInCMPerSec(){
        return this.calculateMaxLinearSpeedInCMPerSec(0);
    }
    public double calculateMaxAngularSpeedInDegPerSec(){
        return this.calculateMaxAngularSpeedInDegPerSec(0);
    }
    public abstract double calculateMaxLinearSpeedInCMPerSec(double angularSpeedInDegPerSec);
    public abstract double calculateMaxAngularSpeedInDegPerSec(double linearSpeedInCMPerSec);
}
