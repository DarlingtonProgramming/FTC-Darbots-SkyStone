package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Util;

import com.acmerobotics.dashboard.config.Config;

import org.darbots.darbotsftclib.libcore.motortypes.servotypes.HS485HB;
import org.darbots.darbotsftclib.libcore.motortypes.servotypes.HS755MG;
import org.darbots.darbotsftclib.libcore.templates.servo_related.ServoType;

@Config
public class ServoPositionCalibrationSettings {
    public static String servoPositionCalibrationConfigName = "leftAutoArmGrabber";
    public static ServoType servoPositionCalibrationServoType = new HS485HB();
    public static double servoPositionCalibrationPos = 0.0;
}
