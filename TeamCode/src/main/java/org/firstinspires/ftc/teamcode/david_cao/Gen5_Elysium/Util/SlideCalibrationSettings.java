package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Util;

import com.acmerobotics.dashboard.config.Config;

import org.darbots.darbotsftclib.libcore.motortypes.GoBilda5202Series117RPMMotor;
import org.darbots.darbotsftclib.libcore.motortypes.GoBilda5202Series435RPMMotor;
import org.darbots.darbotsftclib.libcore.motortypes.RevHDHex40Motor;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;

@Config
public class SlideCalibrationSettings {
    public static String slideCalibrationMotorConfigName = "";
    public static MotorType slideCalibrationMotorType = new GoBilda5202Series117RPMMotor();
    public static double slideCalibrationMinPos = 0;
    public static double slideCalibrationMaxPos = 1000.0;
    public static boolean slideCalibrationReversed = false;
    public static double slideCalibrationSpeed = 0.3;
}
