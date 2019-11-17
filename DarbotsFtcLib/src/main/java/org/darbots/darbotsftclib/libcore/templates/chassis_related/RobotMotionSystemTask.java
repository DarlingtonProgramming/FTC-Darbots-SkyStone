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
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.integratedfunctions.logger.RobotLogFile;
import org.darbots.darbotsftclib.libcore.odometry.Robot2DPositionSoftwareTracker;
import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotGyro;

public abstract class RobotMotionSystemTask implements RobotNonBlockingDevice {
    private RobotMotionSystem m_MotionSystem;
    private boolean m_IsWorking;
    private float m_GyroStartAng = 0.0f;

    public static class MotionSystemTaskFinishInfo{
        public double xError = 0;
        public double yError = 0;
        public double zRotError = 0;
        public double xMoved = 0;
        public double yMoved = 0;
        public double zRotMoved = 0;

        public MotionSystemTaskFinishInfo(){
            xError = 0;
            yError = 0;
            zRotError = 0;
            xMoved = 0;
            yMoved = 0;
            zRotMoved = 0;
        }
        public MotionSystemTaskFinishInfo(double xError, double yError, double ZRotError, double xMoved, double yMoved, double zRotMoved){
            this.xError = xError;
            this.yError = yError;
            this.zRotError = zRotMoved;
            this.xMoved = xMoved;
            this.yMoved = yMoved;
            this.zRotMoved = zRotMoved;
        }
        public MotionSystemTaskFinishInfo(MotionSystemTaskFinishInfo oldInfo){
            this.xError = oldInfo.xError;
            this.yError = oldInfo.yError;
            this.zRotError = oldInfo.zRotError;
            this.xMoved = oldInfo.xMoved;
            this.yMoved = oldInfo.yMoved;
            this.zRotMoved = oldInfo.zRotMoved;
        }

    }

    public RobotMotionSystemTask(){
        this.m_IsWorking = false;
    }
    public RobotMotionSystemTask(@NonNull RobotMotionSystemTask Task){
        this.m_MotionSystem = Task.m_MotionSystem;
        this.m_IsWorking = false;
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
        GlobalUtil.addLog("RobotMotionSystemTask","BeforeTask","", RobotLogFile.LogLevel.DEBUG);
        GlobalUtil.addLog("RobotMotionSystemTask","TaskInfo",this.getTaskDetailString(), RobotLogFile.LogLevel.DEBUG);
        if((!this.getMotionSystem().isCalibrationEnabled()) && GlobalUtil.getGyro() != null){
            RobotGyro globalGyro = GlobalUtil.getGyro();
            globalGyro.updateStatus();
            this.m_GyroStartAng = globalGyro.getHeading();
        } else if(this.getMotionSystem().isCalibrationEnabled()){
            this.m_GyroStartAng = this.getMotionSystem().getGyroGuidedDrivePublicStartingAngle();
        }
        this.__startTask();
    }
    protected abstract void __startTask();
    protected abstract MotionSystemTaskFinishInfo __taskFinished();

    protected float __getGyroStartAng(){
        return this.m_GyroStartAng;
    }

    public void stopTask(){
        if(!this.m_IsWorking){
            return;
        }
        GlobalUtil.addLog("RobotMotionSystemTask","AfterTask","Task ends", RobotLogFile.LogLevel.DEBUG);
        this.m_IsWorking = false;
        MotionSystemTaskFinishInfo finishInfo = this.__taskFinished();
        if(this.m_MotionSystem.getPositionTracker() != null){
            if(this.m_MotionSystem.getPositionTracker() instanceof Robot2DPositionSoftwareTracker){
                Robot2DPositionSoftwareTracker softTracker = (Robot2DPositionSoftwareTracker) this.m_MotionSystem.getPositionTracker();
                softTracker.drive_MoveThroughRobotAxisOffset(new RobotPose2D(
                        finishInfo.xMoved,
                        finishInfo.yMoved,
                        finishInfo.zRotMoved
                ));
            }
        }
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
    public abstract String getTaskDetailString();
    public abstract double getTaskProgressRatio();

    protected double __getGyroGuidedDeltaSpeed(double AbsSpeed){
        if(!this.isBusy()){
            return 0;
        }
        if(GlobalUtil.getGyro() == null || (!this.getMotionSystem().isCalibrationEnabled())){
            return 0;
        }
        RobotGyro globalGyro = GlobalUtil.getGyro();
        globalGyro.updateStatus();
        double currentAng = globalGyro.getHeading();
        double deltaAng = XYPlaneCalculations.normalizeDeg(currentAng - m_GyroStartAng);

        if (globalGyro.getHeadingRotationPositiveOrientation() == RobotGyro.HeadingRotationPositiveOrientation.Clockwise) {
            deltaAng = -deltaAng;
        }
        double absDeltaAng = Math.abs(deltaAng);
        double deltaSpeedEachSide = 0;
        /*
        if (absDeltaAng >= 5) {
            deltaSpeedEachSide = Range.clip(0. * AbsSpeed, 0, 0.2);
        } else if (absDeltaAng >= 1.5) {
            deltaSpeedEachSide = Range.clip(0.2 * AbsSpeed, 0, 0.15);
        } else if (absDeltaAng >= 0.5) {
            deltaSpeedEachSide = Range.clip(0.1 * AbsSpeed, 0, 0.1);
        }
        if (deltaSpeedEachSide < 0.02 && deltaSpeedEachSide != 0) {
            deltaSpeedEachSide = 0.02;
        }
         */
        deltaSpeedEachSide = (-deltaAng) * 0.03; //Proportional Speed Gain
        return deltaSpeedEachSide;
    }
}
