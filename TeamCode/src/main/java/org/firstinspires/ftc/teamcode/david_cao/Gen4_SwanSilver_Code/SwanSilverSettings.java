package org.firstinspires.ftc.teamcode.david_cao.Gen4_SwanSilver_Code;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.motortypes.GoBilda5202Series1150RPMMotor;
import org.darbots.darbotsftclib.libcore.motortypes.MotorTypeUtil;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;

public class SwanSilverSettings {
    //========== Chassis Settings ==========
    public static MotorType CHASSIS_MOTOR_TYPE = MotorTypeUtil.applyGearRatio(
            new GoBilda5202Series1150RPMMotor(),
            2
    );
    public static double CHASSIS_WHEEL_RADIUS = 5.08; //4 INCH DIAMETER
    public static double CHASSIS_LENGTH = 33.6; //14 GOBILDA BIG HOLES. 14*24mm
    public static double CHASSIS_WIDTH = 33.0;
    public static RobotVector2D CHASSIS_LINEAR_FACTORS = new RobotVector2D(
            1.0,
            1.0,
            1.0
    );
    //========== End of Chassis Settings ==========

    //========== Position Tracker Settings ==========

    //========== End of Position Tracker Settings ==========
}
