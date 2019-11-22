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

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.darbots.darbotsftclib.libcore.templates.odometry.Robot2DPositionTracker;

import java.util.ArrayList;

public abstract class RobotMotionSystem implements RobotNonBlockingDevice {
    private ArrayList<RobotMotionSystemTask> m_TaskLists;
    private Robot2DPositionTracker m_PosTracker;
    private double m_LinearYMotionDistanceFactor;
    private double m_LinearXMotionDistanceFactor;
    private double m_RotationalMotionDistanceFactor;
    private boolean m_PosTrackerIsAsync;

    public RobotMotionSystem(Robot2DPositionTracker PositionTracker){
        this.m_TaskLists = new ArrayList();
        this.m_PosTracker = PositionTracker;
        this.setLinearMotionDistanceFactor(1);
        this.m_RotationalMotionDistanceFactor = 1;
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
        if(MotionSystem.m_PosTracker != null && MotionSystem.m_PosTracker instanceof RobotNonBlockingDevice){
            this.m_PosTrackerIsAsync = true;
        }else{
            this.m_PosTrackerIsAsync = false;
        }
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
        if((!this.m_TaskLists.isEmpty())){
            if(this.m_TaskLists.get(0).isBusy())
                this.m_TaskLists.get(0).updateStatus();
        }
        if(this.m_PosTrackerIsAsync){
            RobotNonBlockingDevice NonBlockingTracker = (RobotNonBlockingDevice) this.m_PosTracker;
            NonBlockingTracker.updateStatus();
        }
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


    public void stop(){
        if(this.getPositionTracker() != null){
            this.getPositionTracker().stop();
        }
    }


}
