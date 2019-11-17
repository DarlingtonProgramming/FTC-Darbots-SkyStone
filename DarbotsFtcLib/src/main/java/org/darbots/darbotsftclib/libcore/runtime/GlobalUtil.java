package org.darbots.darbotsftclib.libcore.runtime;

import org.darbots.darbotsftclib.libcore.integratedfunctions.FTCFileIO;
import org.darbots.darbotsftclib.libcore.integratedfunctions.logger.RobotLogFile;
import org.darbots.darbotsftclib.libcore.integratedfunctions.logger.logContents.String_Log;
import org.darbots.darbotsftclib.libcore.templates.log.LogContent;
import org.darbots.darbotsftclib.libcore.templates.log.LogLevel;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotGyro;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;

public class GlobalUtil {
    public static void addLog(String module, String captain, LogContent content, LogLevel logLevel){
        if(GlobalRegister.currentLog != null){
            GlobalRegister.currentLog.addLogContent(module,captain, content, logLevel);
        }
    }
    public static void addLog(String module, String captain, String content, LogLevel logLevel){
        if(GlobalRegister.currentLog != null){
            GlobalRegister.currentLog.addLogContent(module,captain, new String_Log(content), logLevel);
        }
    }

    public static void deleteAllLogs(){
        File[] logFiles = FTCFileIO.getLogFolder().listFiles();
        for(int i=0;i<logFiles.length;i++){
            if(logFiles[i].isFile())
                logFiles[i].delete();
        }
    }

    public static Telemetry getTelemetry(){
        if(GlobalRegister.runningOpMode != null){
            return GlobalRegister.runningOpMode.telemetry;
        }else{
            return null;
        }
    }

    public static RobotGyro getGyro(){
        if(GlobalRegister.runningOpMode != null){
            if(GlobalRegister.runningOpMode.getRobotCore() != null){
                return GlobalRegister.runningOpMode.getRobotCore().getGyro();
            }
        }
        return null;
    }
    public static void OpModeInitialized(){
        return;
    }
    public static void OpModeStarted(){
        if(GlobalRegister.currentLog != null){
            GlobalRegister.currentLog.startOpMode();
        }
    }

    public static void OpModeEnded(){
        if(GlobalRegister.currentLog != null){
            GlobalRegister.currentLog.endOpMode();
        }
    }
}
