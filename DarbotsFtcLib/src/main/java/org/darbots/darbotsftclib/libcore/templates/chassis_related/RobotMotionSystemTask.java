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

import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;

public abstract class RobotMotionSystemTask implements RobotNonBlockingDevice {
    public final static float CONST_TASKSTARTANG_NOTSET = -10000.0f;

    private RobotMotionSystem m_MotionSystem;
    private boolean m_IsWorking;
    private ElapsedTime m_TaskTimer;
    private RobotPose2D m_TaskStartFieldPos = null;
    private RobotPose2D m_TaskRelativeErrorOffset = null;
    private float m_TaskStartAng = CONST_TASKSTARTANG_NOTSET;
    public RobotMotionSystemTaskCallBack TaskCallBack = null;

    public RobotMotionSystemTask(){
        this.m_IsWorking = false;
        this.m_TaskTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }
    public RobotMotionSystemTask(@NonNull RobotMotionSystemTask Task) {
        this.m_MotionSystem = Task.m_MotionSystem;
        this.m_IsWorking = false;
        this.m_TaskTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }
    public RobotMotionSystem getMotionSystem(){
        return this.m_MotionSystem;
    }
    public void setMotionSystem(@NonNull RobotMotionSystem MotionSystem){
        this.m_MotionSystem = MotionSystem;
    }

    public void startTask(){
        if(this.m_IsWorking){
            return;
        }
        this.m_IsWorking = true;
        this.m_TaskStartFieldPos = this.m_MotionSystem.getPositionTracker().getCurrentPosition();
        this.m_TaskRelativeErrorOffset = this.m_MotionSystem.getAccumulatedError();
        this.m_MotionSystem.getPositionTracker().resetRelativeOffset();
        if(this.m_MotionSystem.getGyroValueProvider() != null){
            if(this.m_MotionSystem.getGyroValueProvider() instanceof RobotNonBlockingDevice){
                ((RobotNonBlockingDevice) this.m_MotionSystem.getGyroValueProvider()).updateStatus();
            }
            this.m_TaskStartAng = this.m_MotionSystem.getGyroValueProvider().getHeading();
        }
        this.m_TaskTimer.reset();
        this.__startTask();
    }

    protected abstract void __startTask();
    protected abstract void __taskFinished();
    protected abstract void __updateStatus();
    protected abstract RobotPose2D __getSupposedTaskFinishPos();

    public void stopTask(){
        if(!this.m_IsWorking){
            return;
        }
        this.m_IsWorking = false;
        this.__taskFinished();
        RobotPose2D supposedFinishRelativeOffset = this.__getSupposedTaskFinishPos();
        RobotPose2D RelativePosMoved = this.getRelativePositionOffsetSinceStart();
        if(this.TaskCallBack != null){
            RobotPose2D CurrentPos = this.m_MotionSystem.getPositionTracker().getCurrentPosition();
            this.TaskCallBack.taskFinished(this.m_MotionSystem,this.m_TaskStartFieldPos,CurrentPos,RelativePosMoved);
        }
        RobotPose2D error = new RobotPose2D(
                supposedFinishRelativeOffset.X - RelativePosMoved.X,
                supposedFinishRelativeOffset.Y - RelativePosMoved.Y,
                supposedFinishRelativeOffset.getRotationZ() - RelativePosMoved.getRotationZ()
        );
        this.m_MotionSystem.setAccumulatedError(error);
        this.m_MotionSystem.__checkTasks();
    }
    @Override
    public boolean isBusy(){
        return this.m_IsWorking;
    }
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

    public void updateStatus(){
        this.__updateStatus();
    }

    public double getSecondsSinceTaskStart(){
        return this.m_TaskTimer.seconds();
    }
    protected RobotPose2D getRelativePositionOffsetRawSinceStart(){
        RobotPose2D offset = this.m_MotionSystem.getPositionTracker().getRelativeOffset();
        if(this.m_MotionSystem.getGyroValueProvider() != null && this.m_TaskStartAng != CONST_TASKSTARTANG_NOTSET) {
            double angleMoved = XYPlaneCalculations.normalizeDeg(this.m_MotionSystem.getGyroValueProvider().getHeading() - this.m_TaskStartAng);
            offset.setRotationZ(angleMoved);
        }
        return offset;
    }
    protected RobotPose2D getRelativePositionOffsetSinceStart(){
        RobotPose2D offset = this.getRelativePositionOffsetRawSinceStart();
        offset.X -= this.m_TaskRelativeErrorOffset.X;
        offset.Y -= this.m_TaskRelativeErrorOffset.Y;
        offset.setRotationZ(offset.getRotationZ() - this.m_TaskRelativeErrorOffset.getRotationZ());
        return offset;
    }
    protected RobotPose2D getErrorCorrectionVelocityVector(RobotPose2D supposedPosition){
        double errorX, errorY, errorRotZ;
        RobotPose2D offsetSinceStart = this.getRelativePositionOffsetSinceStart();
        errorX = supposedPosition.X - offsetSinceStart.X;
        errorY = supposedPosition.Y - offsetSinceStart.Y;
        errorRotZ = XYPlaneCalculations.normalizeDeg(supposedPosition.getRotationZ() - offsetSinceStart.getRotationZ());
        this.m_MotionSystem.getPIDCalculator().feedError(errorX,errorY,errorRotZ);
        RobotPose2D correctionVelocity = this.m_MotionSystem.getPIDCalculator().getPIDPower();
        return correctionVelocity;
    }
    protected RobotPose2D setRobotSpeed(RobotPose2D robotSpeed, RobotPose2D supposedRelativePose){
        RobotPose2D correctionVector = this.getErrorCorrectionVelocityVector(supposedRelativePose);
        RobotPose2D afterCorrectionVector = new RobotPose2D(robotSpeed.X + correctionVector.X,robotSpeed.Y + correctionVector.Y,robotSpeed.getRotationZ() + correctionVector.getRotationZ());
        return this.m_MotionSystem.setRobotSpeed(afterCorrectionVector);
    }
}
