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
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.firstinspires.ftc.robotcore.internal.system.RobotApplication;

public abstract class RobotMotionSystemTask implements RobotNonBlockingDevice {
    public final static float CONST_TASKSTARTANG_NOTSET = -10000.0f;

    private RobotMotionSystem m_MotionSystem;
    private boolean m_IsWorking;
    private ElapsedTime m_TaskTimer;
    private RobotPose2D m_TaskActualStartFieldPos = null;
    private RobotPose2D m_TaskSupposedStartFieldPos = null;
    private float m_TaskStartAng = CONST_TASKSTARTANG_NOTSET;
    public RobotMotionSystemTaskCallBack TaskCallBack = null;
    protected RobotPose2D m_LastSupposedPose = null;

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

        this.m_LastSupposedPose = new RobotPose2D(0,0,0);
        this.m_TaskActualStartFieldPos = this.m_MotionSystem.getPositionTracker().getCurrentPosition();
        this.m_TaskSupposedStartFieldPos = this.m_MotionSystem.getLastTaskFinishFieldPos();
        if(this.m_TaskSupposedStartFieldPos == null){
            this.m_TaskSupposedStartFieldPos = new RobotPose2D(this.m_TaskActualStartFieldPos);
        }

        this.m_MotionSystem.getPositionTracker().resetRelativeOffset();

        if(this.m_MotionSystem.getGyroValueProvider() != null){
            if(this.m_MotionSystem.getGyroValueProvider() instanceof RobotNonBlockingDevice){
                ((RobotNonBlockingDevice) this.m_MotionSystem.getGyroValueProvider()).updateStatus();
            }
            this.m_TaskStartAng = this.m_MotionSystem.getGyroValueProvider().getHeading();
        }else{
            this.m_TaskStartAng = CONST_TASKSTARTANG_NOTSET;
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
        RobotPose2D CurrentFieldPos = this.m_MotionSystem.getPositionTracker().getCurrentPosition();

        if(this.TaskCallBack != null){
            this.TaskCallBack.taskFinished(this.m_MotionSystem,this.m_TaskSupposedStartFieldPos,CurrentFieldPos,RelativePosMoved);
        }

        RobotPose2D supposedEndFieldPos = null;
        if(supposedFinishRelativeOffset != null){
            supposedEndFieldPos = XYPlaneCalculations.getAbsolutePosition(this.m_TaskSupposedStartFieldPos,supposedFinishRelativeOffset);
        }else{
            supposedEndFieldPos = CurrentFieldPos;
        }

        this.m_MotionSystem.setLastTaskFinishFieldPos(supposedEndFieldPos);
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
        if(this.isBusy()){
            this.__updateStatus();
        }
    }

    public double getSecondsSinceTaskStart(){
        return this.m_TaskTimer.seconds();
    }
    public int getMSSinceTaskStart(){
        return (int) this.m_TaskTimer.milliseconds();
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
        return calculateErrorCalculatedOffsetPosition(this.getRelativePositionOffsetRawSinceStart());
    }
    private RobotPose2D calculateErrorCalculatedOffsetPosition(RobotPose2D currentRawOffset){
        RobotPose2D currentFieldPos = XYPlaneCalculations.getAbsolutePosition(this.m_TaskActualStartFieldPos, currentRawOffset);
        RobotPose2D errorFixedOffset = XYPlaneCalculations.getRelativePosition(this.m_TaskSupposedStartFieldPos, currentFieldPos);
        return errorFixedOffset;
    }
    protected RobotVector2D getErrorCorrectionVelocityVector(RobotPose2D supposedPosition){
        this.m_LastSupposedPose.setValues(supposedPosition);

        double errorX, errorY, errorRotZ;
        RobotPose2D rawOffsetSinceStart = this.getRelativePositionOffsetRawSinceStart();
        RobotPose2D offsetSinceStart = this.calculateErrorCalculatedOffsetPosition(rawOffsetSinceStart);

        RobotPose2D error = XYPlaneCalculations.getRelativePosition(offsetSinceStart,supposedPosition);
        errorX = error.X;
        errorY = error.Y;
        errorRotZ = error.getRotationZ();

        this.m_MotionSystem.getPIDCalculator().feedError(errorX,errorY,errorRotZ);
        RobotVector2D correctionVelocity = this.m_MotionSystem.getPIDCalculator().getPIDPower();

        return correctionVelocity;
    }
    protected RobotVector2D setRobotSpeed(RobotVector2D robotSpeed, RobotPose2D supposedRelativePose){
        RobotVector2D correctionVector = this.getErrorCorrectionVelocityVector(supposedRelativePose);
        RobotVector2D afterCorrectionVector = new RobotVector2D(robotSpeed.X + correctionVector.X,robotSpeed.Y + correctionVector.Y,robotSpeed.getRotationZ() + correctionVector.getRotationZ());
        RobotVector2D afterCorrectionTheoraticalMaximum = this.m_MotionSystem.getTheoreticalMaximumMotionState(afterCorrectionVector);
        RobotVector2D actualVector = null;
        RobotVector2D returnVector = null;
        if(Math.abs(afterCorrectionVector.X) > Math.abs(afterCorrectionTheoraticalMaximum.X) || Math.abs(afterCorrectionVector.Y) > Math.abs(afterCorrectionTheoraticalMaximum.Y) || Math.abs(afterCorrectionVector.getRotationZ()) > Math.abs(afterCorrectionTheoraticalMaximum.getRotationZ())){
            actualVector = afterCorrectionTheoraticalMaximum;
            double factor = actualVector.X / afterCorrectionVector.X;
            returnVector = new RobotVector2D(robotSpeed.X * factor,robotSpeed.Y * factor, robotSpeed.getRotationZ() * factor);
        }else{
            actualVector = afterCorrectionVector;
            returnVector = new RobotVector2D(robotSpeed);
        }
        this.getMotionSystem().__setRobotSpeed(actualVector.X,actualVector.Y,actualVector.getRotationZ());
        return returnVector;
    }
    public RobotPose2D getLastSupposedPose(){
        return this.m_LastSupposedPose;
    }
}
