package org.darbots.darbotsftclib.libcore.runtime;

import org.darbots.darbotsftclib.libcore.integratedfunctions.logger.RobotLogFile;
import org.darbots.darbotsftclib.libcore.templates.log.LogContent;
import org.darbots.darbotsftclib.libcore.templates.log.LogLevel;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotGyro;

public class GlobalUtil {
    public static void addLog(String module, String captain, LogContent content, LogLevel logLevel){
        if(GlobalRegister.currentLog != null){
            GlobalRegister.currentLog.addLogContent(module,captain, content, logLevel);
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
