package org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.motortypes.AndyMark2964;
import org.darbots.darbotsftclib.libcore.motortypes.AndyMark3637;
import org.darbots.darbotsftclib.libcore.motortypes.GoBilda5202Series1150RPMMotor;
import org.darbots.darbotsftclib.libcore.motortypes.GoBilda5202Series435RPMMotor;
import org.darbots.darbotsftclib.libcore.motortypes.MotorTypeUtil;
import org.darbots.darbotsftclib.libcore.motortypes.encoders.USDigital_E4T_360;
import org.darbots.darbotsftclib.libcore.motortypes.servotypes.HS485HB;
import org.darbots.darbotsftclib.libcore.motortypes.servotypes.HS625MG;
import org.darbots.darbotsftclib.libcore.motortypes.servotypes.HS755HB;
import org.darbots.darbotsftclib.libcore.motortypes.servotypes.HS755MG;
import org.darbots.darbotsftclib.libcore.motortypes.servotypes.HS785HB;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;
import org.darbots.darbotsftclib.libcore.templates.servo_related.ServoType;
import org.darbots.darbotsftclib.season_specific.skystone.SkyStoneCoordinates;

public class LindelSettings {

    //========== Physical Settings ==========
    public static double PHYSICAL_WIDTH = 43.8;
    public static double PHYSICAL_WIDTH_BORDER = 45.5;
    public static double LENGTH_FROM_CENTER_TO_BACK = 19.13;
    public static double LENGTH_FROM_CENTER_TO_FRONT = 45.72-19.13;
    //========== End of Physical Settings ==========

    //========== Chassis Settings ==========
    public static MotorType CHASSIS_MOTOR_TYPE = new AndyMark3637();
    public static double CHASSIS_WHEEL_RADIUS = 5.08; //4 INCH DIAMETER
    public static double CHASSIS_LENGTH = 28.1;
    public static double CHASSIS_WIDTH = 36.5;
    public static RobotVector2D CHASSIS_FACTORS = new RobotVector2D(
            1.0,
            1.0,
            1.0
    );
    //========== End of Chassis Settings ==========

    //========== Position Tracker Settings ==========
    public static RobotVector2D ODOMETRY_FACTORS = new RobotVector2D(
            1.0,
            35.676 / 40,
            171.66 / 180
    );
    //========== End of Position Tracker Settings ==========

    //========== On Robot Component Settings ==========
    public static MotorType LINEAR_SLIDE_MOTORTYPE = new AndyMark2964();
    public static boolean LINEAR_SLIDE_TIMEOUT = true;
    public static double LINEAR_SLIDE_TIMEOUT_FACTOR = 1.5;
    public static double LINEAR_SLIDE_MIN = -0.375, LINEAR_SLIDE_MAX = 6.15, LINEAR_SLIDE_INIT = 0.0, LINEAR_SLIDE_SAFE = 1.58;

    public static double GRABBER_GRAB = 0.1, GRABBER_NOTGRAB = 0.25;
    public static ServoType GRABBER_SERVO_TYPE = new HS755HB();
    public static double GRABBERROT_IN = 0.95, GRABBERROT_OUT = 0.05;
    public static ServoType GRABBERROT_SERVO_TYPE = new HS755HB();

    public static ServoType FOUNDATION_GRABBER_TYPE = new HS485HB();
    public static double FOUNDATION_GRABBER_OFFSET = 0.0;
    public static double FOUNDATION_GRABBER_L_GRAB = (1.0 - FOUNDATION_GRABBER_OFFSET);
    public static double FOUNDATION_GRABBER_L_RELEASED = 0.1; //0.75
    public static double FOUNDATION_GRABBER_L_INIT = 0.1;
    public static double FOUNDATION_GRABBER_R_GRAB = (0.0 + FOUNDATION_GRABBER_OFFSET);
    public static double FOUNDATION_GRABBER_R_RELEASED = 0.9; //0.25
    public static double FOUNDATION_GRABBER_R_INIT = 0.9;

    public static ServoType STONE_ORIENT_SERVO_TYPE = new HS755HB();
    public static double STONE_ORIENT_ZEROPOS = 0.55;
    public static double STONE_ORIENT_ORIENTPOS = 1.0;

    public static final double INTAKEMOTOR_SPEED = 0.8;

    public static final ServoType AUTONOMOUSDRAGSTONESERVO_TYPE = new HS755MG();

    public static final double AUTONOMOUSDRAGSTONESERVO_LEFT_OUTPOS = 0;
    public static final double AUTONOMOUSDRAGSTONESERVO_LEFT_INPOS = 0.6;

    public static final double AUTONOMOUSDRAGSTONESERVO_RIGHT_OUTPOS = 1.0;
    public static final double AUTONOMOUSDRAGSTONESERVO_RIGHT_IN = 0.4;

    public static final double CAPSTONESERVO_INITIALPOS = 1.0;
    public static final double CAPSTONESERVO_DEPOSITPOS = 0.0;
    public static final ServoType CAPSTONESERVO_SERVO_TYPE = new HS485HB();
    //========== End of On Robot Component Settings ==========

    public static double CONTROL_STICK_THRESHOLD = 0.2;
    public static double CONTROL_SLIDE_MAXSPEED = 1.0;
    public static double CONTROL_CHASSIS_MAXSPEED_NORMALIZED = 1.0;
    public static double CONTROL_STONE_HEIGHT_SLIDE = 1.09464;
    public static double CONTROL_STONE_PROMINENCE_HEIGHT_SLIDE = 0.36429;
    public static double CONTROL_COMBO_STONE_INITILAL_HEIGHT = LINEAR_SLIDE_MIN - CONTROL_STONE_HEIGHT_SLIDE;
    public static double CONTROL_COMBO_MAXIMUM_SPEED_NORMALIZED = 1.0;
    public static double CONTROL_COMBO_MAXIMUM_ACCEL_NORMALIZED = 0.2;
    public static double CONTROL_COMBO_EXIT_FORWARD_DIST = SkyStoneCoordinates.STONE_LENGTH * 1.5;
}
