package org.darbots.darbotsftclib.libcore.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.integratedfunctions.logger.RobotLogFile;
import org.darbots.darbotsftclib.libcore.integratedfunctions.logger.logContents.String_Log;
import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;
import org.darbots.darbotsftclib.libcore.templates.log.LogLevel;

public abstract class DarbotsBasicOpMode<CoreType extends RobotCore> extends LinearOpMode {
    private ElapsedTime m_TimerSinceStart = null;
    private ElapsedTime m_TimerSinceInit = null;
    public abstract CoreType getRobotCore();
    public abstract void hardwareInitialize();
    public abstract void hardwareDestroy();
    public abstract void RunThisOpMode();
    @Override
    public void runOpMode() throws InterruptedException {
        GlobalRegister.runningOpMode = this;
        this.m_TimerSinceStart = null;
        this.m_TimerSinceInit = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        this.hardwareInitialize();
        GlobalUtil.addLog("DarbotsBasicOpMode","Status",new String_Log("OpMode initialized"), LogLevel.DEBUG);
        GlobalUtil.OpModeInitialized();
        this.waitForStart();
        if(this.opModeIsActive()){
            GlobalUtil.OpModeStarted();
            this.m_TimerSinceStart = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            RunThisOpMode();
        }
        GlobalUtil.addLog("DarbotsBasicOpMode","Status",new String_Log("OpMode stopping"), LogLevel.DEBUG);

        if(this.getRobotCore() != null) {
            this.getRobotCore().stop();
            this.getRobotCore().terminate();
        }

        GlobalUtil.addLog("DarbotsBasicOpMode","Status",new String_Log("OpMode finished"), LogLevel.DEBUG);
        GlobalUtil.OpModeEnded();
        if(this.getRobotCore().getLogger() != null) {
            this.getRobotCore().getLogger().saveToFile();
        }
        this.hardwareDestroy();
        GlobalRegister.runningOpMode = null;
        GlobalRegister.currentLog = null;
        this.m_TimerSinceStart = null;
        this.m_TimerSinceInit = null;
    }
    @Override
    public void waitForStart(){
        while ((!opModeIsActive()) && (!isStopRequested())) {
            telemetry.addData("status", "Initialized, waiting for start command...");
            telemetry.update();
        }
        if(opModeIsActive()){
            telemetry.addData("status","Started");
            telemetry.update();
        }
    }

    public void waitForStart_NoDisplay(){
        super.waitForStart();
    }

    public boolean waitForDrive(){
        while(this.opModeIsActive() && this.getRobotCore().getChassis().isBusy()){
            this.getRobotCore().updateStatus();
        }
        return this.opModeIsActive();
    }

    public boolean delay(double seconds){
        ElapsedTime m_Time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        m_Time.reset();
        while(this.opModeIsActive() && m_Time.seconds() < seconds){
            sleep(20);
        }
        return this.opModeIsActive();
    }

    public double getSecondsSinceOpModeStarted(){
        if(this.m_TimerSinceStart == null){
            return 0;
        }else{
            return this.m_TimerSinceStart.seconds();
        }
    }

    public double getSecondsSinceOpModeInited(){
        if(this.m_TimerSinceInit == null){
            return 0;
        }else{
            return this.m_TimerSinceInit.seconds();
        }
    }
}
