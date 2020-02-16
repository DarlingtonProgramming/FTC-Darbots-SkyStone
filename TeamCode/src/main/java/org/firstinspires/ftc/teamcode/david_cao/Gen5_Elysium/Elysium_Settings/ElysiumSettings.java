package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.motortypes.GoBilda5202Series1150RPMMotor;
import org.darbots.darbotsftclib.libcore.motortypes.GoBilda5202Series435RPMMotor;
import org.darbots.darbotsftclib.libcore.motortypes.MotorTypeUtil;
import org.darbots.darbotsftclib.libcore.motortypes.RevHDHex40Motor;
import org.darbots.darbotsftclib.libcore.motortypes.servotypes.HS485HB;
import org.darbots.darbotsftclib.libcore.motortypes.servotypes.HS755HB;
import org.darbots.darbotsftclib.libcore.motortypes.servotypes.HS755MG;
import org.darbots.darbotsftclib.libcore.motortypes.servotypes.HS765HB;
import org.darbots.darbotsftclib.libcore.motortypes.servotypes.HS785HB;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;
import org.darbots.darbotsftclib.libcore.templates.servo_related.ServoType;

public class ElysiumSettings {
    //========== Start of Physical Settings ==========
    public static final double PHYSICAL_MASS_KG = 6.0;
    public static final double PHYSICAL_CENTER_TO_FRONT = 21.5; //Exactly 18 Inches
    public static final double PHYSICAL_CENTER_TO_FRONT_EXPANDED = 26.0;
    public static final double PHYSICAL_CENTER_TO_BACK = 23.0;
    public static final double PHYSICAL_CENTER_TO_BACK_DOOR_CLOSED = 35.5;
    public static final double PHYSICAL_CENTER_TO_LEFT = 19.0;
    public static final double PHYSICAL_CENTER_TO_LEFT_SIGN = 18.7;
    public static final double PHYSICAL_CENTER_TO_RIGHT = 20.8;
    public static final double PHYSICAL_CENTER_TO_RIGHT_SIGN = 20.8;
    public static final double PHYSICAL_CENTER_TO_LEFT_DOOR = 22.0;
    public static final double PHYSICAL_CENTER_TO_RIGHT_DOOR = 22.0;
    public static final double PHYSICAL_WIDTH = PHYSICAL_CENTER_TO_LEFT + PHYSICAL_CENTER_TO_RIGHT;
    public static final double PHYSICAL_LENGTH = PHYSICAL_CENTER_TO_FRONT_EXPANDED + PHYSICAL_CENTER_TO_BACK;
    public static final double PHYSICAL_INTERTIA = (PHYSICAL_MASS_KG * (Math.pow(PHYSICAL_LENGTH / 2.0 / 100.0,2) + Math.pow(PHYSICAL_WIDTH / 2.0 / 100.0,2))) / 2.0; //Suppose Mass evenly distributed, MR^2 / 2.0, unit in kg*m*m
    //========== End of Physical Settings ==========
    //========== Start of Chassis Settings ==========
    public static final MotorType CHASSIS_MOTOR_TYPE = MotorTypeUtil.applyGearRatio(
            new GoBilda5202Series1150RPMMotor(),
            2.0
    );
    public static final double CHASSIS_MOTOR_TORQUE_KG_PER_CM = 7.9;
    public static final double CHASSIS_WHEEL_RADIUS = 5.08; //4 Inch Diameter
    public static final double CHASSIS_LENGTH = 31.2; //14 GOBILDA BIG HOLES 14 * 24mm
    public static final double CHASSIS_WIDTH = 33.0;
    public static final RobotVector2D CHASSIS_FACTORS = new RobotVector2D(
            1.0,
            1.0,
            1.0
    );
    public static final double CHASSIS_FRICTION_FACTOR = 0.6;
    public static final double CHASSIS_MAXIMUM_ACCEL = (((CHASSIS_MOTOR_TORQUE_KG_PER_CM * 9.8 * 2.0 / CHASSIS_WHEEL_RADIUS) * 4) / PHYSICAL_MASS_KG * 100.0) * CHASSIS_FRICTION_FACTOR; //7.9 kg * cm with a friction factor
    public static final double CHASSIS_MAXIMUM_ANGULAR_ACCEL_RAD = (((CHASSIS_MOTOR_TORQUE_KG_PER_CM * 9.8 * 2.0 / CHASSIS_WHEEL_RADIUS) * 4) * Math.sqrt(Math.pow(CHASSIS_LENGTH / 2.0 / 100.0,2) + Math.pow(CHASSIS_WIDTH/2.0 / 100.0,2)) / PHYSICAL_INTERTIA) * CHASSIS_FRICTION_FACTOR;
    public static final double CHASSIS_MAXIMUM_ANGULAR_ACCEL_DEG = Math.toDegrees(CHASSIS_MAXIMUM_ANGULAR_ACCEL_RAD);
    //========== End of Chassis Settings ==========

    //========== Start of Position Localization Settings ==========
    public static final RobotPoint2D LOCALIZATION_LEFTDISTSENSOR_POS = new RobotPoint2D(13.0,38.5/2);
    public static final RobotPoint2D LOCALIZATION_RIGHTDISTSENSOR_POS = new RobotPoint2D(13.0,-38.5/2);
    public static final RobotPoint2D LOCALIZATION_FRONTDISTSENSOR_POS = new RobotPoint2D(18.3,0);
    public static final RobotPoint2D LOCALIZATION_BACKDISTSENSOR_POS = new RobotPoint2D(-20.3,0);
    //========== End of Position Localization Settings ==========

    //========== Start of Autonomous Claw Settings ==========
    public static final ServoType AUTONOMOUS_CLAW_ROTSERVO_TYPE = new HS755MG();
    public static final ServoType AUTONOMOUS_CLAW_GRABSERVO_TYPE = new HS485HB();
    public static final RobotPoint2D AUTONOMOUS_CLAW_LEFT_POSITION_WHEN_DOWN = new RobotPoint2D(13.5,24.9);
    public static final double AUTONOMOUS_CLAW_LEFT_INSIDE_POS = 0.15;
    public static final double AUTONOMOUS_CLAW_LEFT_ROT_STONE_GRABBED_POS = 0.41;
    public static final double AUTONOMOUS_CLAW_LEFT_REST_POS = 0.28;
    public static final double AUTONOMOUS_CLAW_LEFT_OUTSIDE_POS = 0.89;
    public static final double AUTONOMOUS_CLAW_LEFT_WIDE_OPEN_POS = 0.8;
    public static final double AUTONOMOUS_CLAW_LEFT_ALL_CLOSED_POS = 0.0;
    public static final double AUTONOMOUS_CLAW_LEFT_GRAB_STONE_POS = 0.1;

    public static final RobotPoint2D AUTONOMOUS_CLAW_RIGHT_POSITION_WHEN_DOWN = new RobotPoint2D(13.5,-24.9);
    public static final double AUTONOMOUS_CLAW_RIGHT_INSIDE_POS = 0.9;
    public static final double AUTONOMOUS_CLAW_RIGHT_ROT_STONE_GRABBED_POS = 0.56;
    public static final double AUTONOMOUS_CLAW_RIGHT_REST_POS = 0.75;
    public static final double AUTONOMOUS_CLAW_RIGHT_OUTSIDE_POS = 0.11;
    public static final double AUTONOMOUS_CLAW_RIGHT_WIDE_OPEN_POS = 0.0;
    public static final double AUTONOMOUS_CLAW_RIGHT_ALL_CLOSED_POS = 0.7;
    public static final double AUTONOMOUS_CLAW_RIGHT_GRAB_STONE_POS = 0.6;

    public static final double AUTONOMOUS_CLAW_WAIT_FOR_CLOSE_SEC = 0.1;
    public static final double AUTONOMOUS_CLAW_WAIT_FOR_OUT_IN_SEC = 0.5;
    //========== End of Autonomous Claw Settings ==========

    //========== Start of Stacker System Settings ==========
    public static final double STACKER_SLIDE_MIN_POS = 0;
    public static final double STACKER_ABOVE_FOUNDATION_POS = 0;
    public static final double STACKER_STONE_DOWN_POS = 0;
    public static final double STACKER_SLIDE_MAX_POS = 0;
    public static final MotorType STACKER_SLIDE_TYPE = new GoBilda5202Series435RPMMotor();

    public static final double STACKER_DOOR_LEFT_OPEN_POS = 0;
    public static final double STACKER_DOOR_LEFT_CLOSED_POS = 0;
    public static final double STACKER_DOOR_RIGHT_OPEN_POS = 0;
    public static final double STACKER_DOOR_RIGHT_CLOSED_POS = 0;
    public static final ServoType STACKER_DOOR_SERVO_TYPE = new HS755MG();
    //========== End of Stacker System Settings ==========

    //========== Start of Out-Take System Settings ==========
    public static final double OUTTAKE_SLIDE_MIN_POS = 0.78;
    public static final double OUTTAKE_SLIDE_MAX_POS = 0.08;
    public static final double OUTTAKE_SLIDE_OUT_OF_WAY_INTAKE_POS = 0.6;
    public static final double OUTTAKE_SLIDE_SAFE_FAR_RELEASE = 0.3;
    public static final ServoType OUTTAKE_SLIDE_SERVO_TYPE = new HS755MG();
    public static final double OUTTAKE_GRABBER_FAR_RELEASED_POS = 0.6;
    public static final double OUTTAKE_GRABBER_GRABBED_POS = 0.25;
    public static final double OUTTAKE_GRABBER_RELEASED_POS = 0.4;
    public static final ServoType OUTTAKE_GRABBER_SERVO_TYPE = new HS485HB();
    public static final double OUTTAKE_GRABBER_WAIT_SEC = 0.5;
    //========== End of Our-Take System Settings ==========

    //========== Start of Intake System Settings ==========
    public static final double INTAKE_POSITIONING_SERVO_REST = 0.0;
    public static final double INTAKE_POSITIONING_SERVO_AUTO = 0.0;
    public static final double INTAKE_POSITIONING_SERVO_HIT = 0.0;
    public static final ServoType INTAKE_POSITIONING_SERVO_TYPE = new HS485HB();
    public static final double INTAKE_POSITIONING_SERVO_WAIT_SEC = 0.3;
    //========== End of Intake System Settings ==========

    //========== Start of Capstone Delivery System Settings ==========
    public static final double CAPSTONE_SLIDE_MIN = 0;
    public static final double CAPSTONE_SLIDE_MAX = 8.5;
    public static final MotorType CAPSTONE_SLIDE_MOTOR_TYPE = new RevHDHex40Motor();

    public static final double CAPSTONE_SERVO_IN_POS = 0.785;
    public static final double CAPSTONE_SERVO_OUT_POS = 0.32;
    public static final ServoType CAPSTONE_SERVO_TYPE = new HS485HB();
    //========== End of Capstone Delivery System Settings ==========
}
