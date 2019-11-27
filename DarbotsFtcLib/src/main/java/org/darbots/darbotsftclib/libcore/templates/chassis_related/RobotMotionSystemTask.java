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
import com.qualcomm.robotcore.util.Range;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.integratedfunctions.logger.RobotLogFile;
import org.darbots.darbotsftclib.libcore.odometry.Robot2DPositionSoftwareTracker;
import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotMotion;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.darbots.darbotsftclib.libcore.templates.log.LogLevel;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotGyro;
import org.firstinspires.ftc.robotcore.internal.android.dx.io.instructions.ZeroRegisterDecodedInstruction;

public abstract class RobotMotionSystemTask implements RobotNonBlockingDevice {
    private RobotMotionSystem m_MotionSystem;
    private boolean m_IsWorking;
    private static final double LASTSPEED_NOTSET = -100000;
    private static final double SPEEDFACTOR_NONE = -100000;
    private double m_LastXSpeed = LASTSPEED_NOTSET, m_LastYSpeed = LASTSPEED_NOTSET, m_LastRotZSpeed = LASTSPEED_NOTSET;
    private ElapsedTime m_SpeedControlTimer;
    private ElapsedTime m_TaskTimer;

    public RobotMotionSystemTask(){
        this.m_IsWorking = false;
        this.m_SpeedControlTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        this.m_TaskTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }
    public RobotMotionSystemTask(@NonNull RobotMotionSystemTask Task) {
        this.m_MotionSystem = Task.m_MotionSystem;
        this.m_IsWorking = false;
        this.m_SpeedControlTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
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
        this.m_TaskTimer.reset();
        this.__startTask();
    }
    protected abstract void __startTask();
    protected abstract RobotPose2D __taskFinished();

    public void stopTask(){
        if(!this.m_IsWorking){
            return;
        }
        this.m_IsWorking = false;
        RobotPose2D finishPose = this.__taskFinished();
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


    private RobotPose2D getCorrectionVelocity(RobotPose2D supposedPosition){
        RobotPose2D errorVals = XYPlaneCalculations.getRelativePosition(this.getMotionSystem().getPositionTracker().getCurrentPosition(),supposedPosition);
        this.getMotionSystem().getPIDCalculator().feedError(errorVals);
        RobotPose2D newDeltaVelocity = this.getMotionSystem().getPIDCalculator().getPDPower();
        return newDeltaVelocity;
    }


    public double getSecondsSinceTaskStart(){
        return this.m_TaskTimer.seconds();
    }

}
