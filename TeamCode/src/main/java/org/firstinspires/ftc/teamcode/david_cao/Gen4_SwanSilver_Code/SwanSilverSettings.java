package org.firstinspires.ftc.teamcode.david_cao.Gen4_SwanSilver_Code;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.motortypes.GoBilda5202Series1150RPMMotor;
import org.darbots.darbotsftclib.libcore.motortypes.MotorTypeUtil;
import org.darbots.darbotsftclib.libcore.motortypes.encoders.USDigital_E4T_360;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotWheel;
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
            165.54 / 180.0
    );
    //========== End of Chassis Settings ==========

    //========== Position Tracker Settings ==========
    public static MotorType ODOMETRY_MOTOR_TYPE = new USDigital_E4T_360();
    public static RobotPose2D ODOMETRY_ENCODER1_WHEEL_POSITION = new RobotPose2D(
            0,
            0,
            0
    );
    public static RobotPose2D ODOMETRY_ENCODER2_WHEEL_POSITION = new RobotPose2D(
            0,
            0,
            0
    );
    public static boolean ODOMETRY_ENCODER1_REVERSED = false;
    public static boolean ODOMETRY_ENCODER2_REVERSED = false;
    public static double ODOMETRY_WHEEL_RADIUS = 1.9; //38mm RobotShop Wheel
    //========== End of Position Tracker Settings ==========
}
