package org.darbots.darbotsftclib.libcore.templates;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.darbots.darbotsftclib.libcore.integratedfunctions.logger.RobotLogFile;
import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.sensors.gyros.BNO055Gyro;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;
import org.darbots.darbotsftclib.libcore.templates.log.LogLevel;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotGyro;

/**
 * This abstract class is used by the programmers to define components on iterations of robots
 * The RobotCore class includes a Logger, a Gyro (can be an actual IMU on extension hub or PositionTracker-Powered Gyros), and a MotionSystem at least.
 * Every Component of the Robot's updateStatus() method and isBusy() method should be nested inside RobotCore's UpdateStatus() and isBusy() class.
 * @author David Cao
 * @see org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice
 */
public abstract class RobotCore implements RobotNonBlockingDevice {
    private RobotLogFile m_Logger;
    private RobotGyro m_Gyro;
    private long m_UpdateStatusCount = 0;
    public RobotCore(String logFileName, HardwareMap hardwareMap){
        if(logFileName != null && (!logFileName.isEmpty())) {
            m_Logger = new RobotLogFile(logFileName);
            GlobalRegister.currentLog = m_Logger.addNewRunLog();
        }else{
            m_Logger = null;
            GlobalRegister.currentLog = null;
        }
        GlobalUtil.LowestLogLevel = LogLevel.INFO;
        m_Gyro = new BNO055Gyro(hardwareMap,"imu");
        this.m_UpdateStatusCount = 0;
    }
    public RobotCore(String logFileName, HardwareMap hardwareMap, int ThreadPriority){
        if(logFileName != null && (!logFileName.isEmpty())) {
            m_Logger = new RobotLogFile(logFileName);
            GlobalRegister.currentLog = m_Logger.addNewRunLog();
        }else{
            m_Logger = null;
            GlobalRegister.currentLog = null;
        }
        GlobalUtil.LowestLogLevel = LogLevel.INFO;
        m_Gyro = new BNO055Gyro(hardwareMap,"imu");
        if(ThreadPriority >= Thread.MIN_PRIORITY || ThreadPriority <= Thread.MAX_PRIORITY){
            Thread.currentThread().setPriority(ThreadPriority);
            GlobalRegister.currentLog.threadPriority = ThreadPriority;
        }
        this.m_UpdateStatusCount = 0;
    }
    public void stop(){
        this.__stop();
    }
    public void terminate(){
        this.__terminate();
        GlobalUtil.addLog(this.getClass().getSimpleName(),"UpdateStatus Count",this.m_UpdateStatusCount, LogLevel.INFO);
        if(GlobalRegister.runningOpMode != null){
            GlobalUtil.addLog(this.getClass().getSimpleName(),"UpdateStatus Frequency",this.m_UpdateStatusCount / GlobalRegister.runningOpMode.getSecondsSinceOpModeStarted(),LogLevel.INFO);
        }
    }
    protected abstract void __stop();
    protected abstract void __terminate();
    public RobotLogFile getLogger(){
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
    protected abstract void __updateStatus();
    @Override
    public void updateStatus(){
        this.m_UpdateStatusCount++;
        this.__updateStatus();
    }
    public long getUpdateStatusCount(){
        return this.m_UpdateStatusCount;
    }
    public abstract void updateTelemetry();
    public RobotGyro getGyro(){
        return this.m_Gyro;
    }
}
