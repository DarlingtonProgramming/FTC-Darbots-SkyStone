package org.firstinspires.ftc.teamcode.david_cao.Gen4_SwanSilver_Code;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.motortypes.GoBilda5202Series1150RPMMotor;
import org.darbots.darbotsftclib.libcore.motortypes.GoBilda5202Series435RPMMotor;
import org.darbots.darbotsftclib.libcore.motortypes.MotorTypeUtil;
import org.darbots.darbotsftclib.libcore.motortypes.encoders.USDigital_E4T_360;
import org.darbots.darbotsftclib.libcore.motortypes.servotypes.HS625MG;
import org.darbots.darbotsftclib.libcore.motortypes.servotypes.HS755MG;
import org.darbots.darbotsftclib.libcore.motortypes.servotypes.HS785HB;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;
import org.darbots.darbotsftclib.libcore.templates.servo_related.ServoType;

public class SwanSilverSettings {

    public static double CONTROL_DRIVE_THRESEHOLD = 0.2;
    public static double CONTROL_STICK_THRESHOLD = 0.2;

    //========== Chassis Settings ==========
    public static MotorType CHASSIS_MOTOR_TYPE = MotorTypeUtil.applyGearRatio(
            new GoBilda5202Series1150RPMMotor(),
            2
    );
    public static double CHASSIS_WHEEL_RADIUS = 5.08; //4 INCH DIAMETER
    public static double CHASSIS_LENGTH = 33.6; //14 GOBILDA BIG HOLES. 14*24mm
    public static double CHASSIS_WIDTH = 33.0;
    public static RobotVector2D CHASSIS_FACTORS = new RobotVector2D(
            1.0,
            27.568 / 30.0,
            175.847 / 180.0
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
    public static RobotVector2D ODOMETRY_FACTORS = new RobotVector2D(
            1.0,
            1.0,
            1.0
    );
    //========== End of Position Tracker Settings ==========

    //========== On Robot Component Settings ==========
    public static MotorType LINEAR_SLIDE_MOTORTYPE = new GoBilda5202Series435RPMMotor();
    public static boolean LINEAR_SLIDE_TIMEOUT = true;
    public static double LINEAR_SLIDE_TIMEOUT_FACTOR = 2.0;
    public static double LINEAR_SLIDE_MIN = 0.0, LINEAR_SLIDE_MAX = 1000.0, LINEAR_SLIDE_INIT = 0.0;

    public static double GRABBER_GRAB = 0, GRABBER_NOTGRAB = 0.7;
    public static ServoType GRABBER_SERVO_TYPE = new HS625MG();
    public static double GRABBERMOVER_MIN = 0.14, GRABBERMOVER_MAX = 0.65, GRABBERMOVER_MEDIUM = (GRABBERMOVER_MIN + GRABBERMOVER_MAX) / 2.0;
    public static ServoType GRABBERMOVER_SERVO_TYPE = new HS785HB();

    public static ServoType FOUNDATION_GRABBER_TYPE = new HS755MG();
    public static double FOUNDATION_GRABBER_OFFSET = 0.1;
    public static double FOUNDATION_GRABBER_L_GRAB = (1.0 - FOUNDATION_GRABBER_OFFSET);
    public static double FOUNDATION_GRABBER_L_RELEASED = 0.0;
    public static double FOUNDATION_GRABBER_R_GRAB = (0.0 + FOUNDATION_GRABBER_OFFSET);
    public static double FOUNDATION_GRABBER_R_RELEASED = 1.0;
    //========== End of On Robot Component Settings ==========
}
