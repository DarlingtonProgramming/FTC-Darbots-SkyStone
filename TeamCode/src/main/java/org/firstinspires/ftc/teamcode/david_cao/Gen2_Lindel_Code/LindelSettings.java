package org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.motortypes.AndyMark2964;
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

public class LindelSettings {

    public static double CONTROL_STICK_THRESHOLD = 0.2;
    public static double CONTROL_SLIDE_MAXSPEED = 1.0;
    public static double CONTROL_CHASSIS_MAXSPEED_NORMALIZED = 1.0;

    //========== Chassis Settings ==========
    public static MotorType CHASSIS_MOTOR_TYPE = MotorTypeUtil.applyGearRatio(
            new GoBilda5202Series1150RPMMotor(),
            2
    );
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
            1.0,
            1.0
    );
    //========== End of Position Tracker Settings ==========

    //========== On Robot Component Settings ==========
    public static MotorType LINEAR_SLIDE_MOTORTYPE = new AndyMark2964();
    public static boolean LINEAR_SLIDE_TIMEOUT = true;
    public static double LINEAR_SLIDE_TIMEOUT_FACTOR = 1.5;
    public static double LINEAR_SLIDE_MIN = -1000.0, LINEAR_SLIDE_MAX = 1000.0, LINEAR_SLIDE_INIT = 0.0;

    public static double GRABBER_GRAB = 0.1, GRABBER_NOTGRAB = 0.25;
    public static ServoType GRABBER_SERVO_TYPE = new HS755HB();
    public static double GRABBERROT_IN = 0.95, GRABBERROT_OUT = 0.05;
    public static ServoType GRABBERROT_SERVO_TYPE = new HS755HB();

    public static ServoType FOUNDATION_GRABBER_TYPE = new HS485HB();
    public static double FOUNDATION_GRABBER_OFFSET = 0.0;
    public static double FOUNDATION_GRABBER_L_GRAB = (1.0 - FOUNDATION_GRABBER_OFFSET);
    public static double FOUNDATION_GRABBER_L_RELEASED = 0.2;
    public static double FOUNDATION_GRABBER_R_GRAB = (0.0 + FOUNDATION_GRABBER_OFFSET);
    public static double FOUNDATION_GRABBER_R_RELEASED = 0.8;

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
}
