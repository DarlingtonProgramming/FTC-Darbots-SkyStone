package org.darbots.darbotsftclib.libcore.runtime;

import org.darbots.darbotsftclib.libcore.integratedfunctions.logger.RobotLogger;
import org.darbots.darbotsftclib.libcore.templates.odometry.Robot2DPositionTracker;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotGyro;

public class GlobalUtil {
    public static void addLog(String module, String captain, Object content, RobotLogger.LogLevel logLevel){
        if(GlobalRegister.runningOpMode != null){
            if(GlobalRegister.runningOpMode.getRobotCore() != null)
                GlobalRegister.runningOpMode.getRobotCore().getLogger().addLog(module,captain,content, logLevel);
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
}
