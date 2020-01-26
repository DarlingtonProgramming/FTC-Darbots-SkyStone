package org.darbots.darbotsftclib.libcore_4_5_0Pre.runtime;

import com.qualcomm.robotcore.util.RobotLog;

import org.darbots.darbotsftclib.libcore_4_5_0Pre.integratedfunctions.FTCFileIO;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.integratedfunctions.logger.logContents.Number_Log;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.integratedfunctions.logger.logContents.String_Log;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.templates.log.LogContent;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.templates.log.LogLevel;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.templates.other_sensors.RobotGyro;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;

public class GlobalUtil {
    public static LogLevel LowestLogLevel = LogLevel.INFO;
    public static void addLog(String module, String caption, LogContent content, LogLevel logLevel){
        if(GlobalRegister.currentLog != null && logLevel.value() >= LowestLogLevel.value()){
            LogContent logContent = content;
            GlobalRegister.currentLog.addLogContent(module,caption,logContent,logLevel);
            debugOutputLog(module,caption,logContent,logLevel);
        }
    }
    public static void addLog(String module, String caption, String content, LogLevel logLevel){
        if(GlobalRegister.currentLog != null && logLevel.value() >= LowestLogLevel.value()){
            LogContent logContent = new String_Log(content);
            GlobalRegister.currentLog.addLogContent(module,caption,logContent,logLevel);
            debugOutputLog(module,caption,logContent,logLevel);
        }
    }
    public static void addLog(String module, String caption, Number content, LogLevel logLevel){
        if(GlobalRegister.currentLog != null && logLevel.value() >= LowestLogLevel.value()){
            LogContent logContent = new Number_Log(content);
            GlobalRegister.currentLog.addLogContent(module,caption,logContent,logLevel);
            debugOutputLog(module,caption,logContent,logLevel);
        }
    }

    public static void debugOutputLog(String module, String caption, LogContent logContent, LogLevel logLevel){
        System.out.print("[" + logLevel.name() + "]");
        System.out.print(module + "::" + caption);
        System.out.println(" | " + logContent.getContentValue().toString());
        RobotLog.dd(module,caption + ":" + logContent.getContentValue().toString());
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
