package org.darbots.darbotsftclib.libcore.templates;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.darbots.darbotsftclib.libcore.integratedfunctions.logger.RobotLogger;
import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.sensors.gyros.BNO055Gyro;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotGyro;

import android.os.Process;

/**
 * This abstract class is used by the programmers to define components on iterations of robots
 * The RobotCore class includes a Logger, a Gyro (can be an actual IMU on extension hub or PositionTracker-Powered Gyros), and a MotionSystem at least.
 * Every Component of the Robot's updateStatus() method and isBusy() method should be nested inside RobotCore's UpdateStatus() and isBusy() class.
 * @author David Cao
 * @see org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice
 */
public abstract class RobotCore implements RobotNonBlockingDevice {
    private RobotLogger m_Logger;
    private RobotGyro m_Gyro;
    public RobotCore(String logFileName, HardwareMap hardwareMap){
        m_Logger = new RobotLogger(logFileName);
        m_Gyro = new BNO055Gyro(hardwareMap,"imu");
    }
    public RobotCore(String logFileName, HardwareMap hardwareMap, int ThreadPriority){
        m_Logger = new RobotLogger(logFileName);
        m_Gyro = new BNO055Gyro(hardwareMap,"imu");
        if(ThreadPriority >= Thread.MIN_PRIORITY || ThreadPriority <= Thread.MAX_PRIORITY){
            Thread.currentThread().setPriority(ThreadPriority);
        }
    }
    public abstract void stop();
    public abstract void terminate();
    public RobotLogger getLogger(){
        return this.m_Logger;
    }
    public abstract RobotMotionSystem getChassis();
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
    public abstract void updateTelemetry();
    public RobotGyro getGyro(){
        return this.m_Gyro;
    }
}
