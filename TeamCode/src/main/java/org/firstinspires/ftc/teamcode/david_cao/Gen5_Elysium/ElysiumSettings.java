package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.motortypes.GoBilda5202Series1150RPMMotor;
import org.darbots.darbotsftclib.libcore.motortypes.GoBilda5202Series435RPMMotor;
import org.darbots.darbotsftclib.libcore.motortypes.MotorTypeUtil;
import org.darbots.darbotsftclib.libcore.motortypes.RevHDHex40Motor;
import org.darbots.darbotsftclib.libcore.motortypes.servotypes.HS755HB;
import org.darbots.darbotsftclib.libcore.motortypes.servotypes.HS755MG;
import org.darbots.darbotsftclib.libcore.motortypes.servotypes.HS765HB;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;
import org.darbots.darbotsftclib.libcore.templates.servo_related.ServoType;

public class ElysiumSettings {
    //========== Start of Chassis Settings ==========
    public static final MotorType CHASSIS_MOTOR_TYPE = MotorTypeUtil.applyGearRatio(
            new GoBilda5202Series1150RPMMotor(),
            2.0
    );
    public static final double CHASSIS_WHEEL_RADIUS = 5.08; //4 Inch Diameter
    public static final double CHASSIS_LENGTH = 33.6; //14 GOBILDA BIG HOLES 14 * 24mm
    public static final double CHASSIS_WIDTH = 33.0;
    public static final RobotVector2D CHASSIS_FACTORS = new RobotVector2D(
            1.0,
            1.0,
            1.0
    );
    //========== End of Chassis Settings ==========

    //========== Start of Position Localization Settings ==========
    public static final RobotPoint2D LOCALIZATION_LEFTDISTSENSOR_POS = new RobotPoint2D(0,0);
    public static final RobotPoint2D LOCALIZATION_RIGHTDISTSENSOR_POS = new RobotPoint2D(0,0);
    public static final RobotPoint2D LOCALIZATION_FRONTDISTSENSOR_POS = new RobotPoint2D(0,0);
    public static final RobotPoint2D LOCALIZATION_BACKDISTSENSOR_POS = new RobotPoint2D(0,0);
    //========== End of Position Localization Settings ==========

    //========== Start of Autonomous Claw Settings ==========
    public static final ServoType AUTONOMOUS_CLAW_ROTSERVO_TYPE = new HS755MG();
    public static final ServoType AUTONOMOUS_CLAW_GRABSERVO_TYPE = new HS765HB();
    public static final RobotPoint2D AUTONOMOUS_CLAW_LEFT_POSITION_WHEN_DOWN = new RobotPoint2D(0,0);
    public static final double AUTONOMOUS_CLAW_LEFT_INSIDE_POS = 0;
    public static final double AUTONOMOUS_CLAW_LEFT_REST_POS = 0;
    public static final double AUTONOMOUS_CLAW_LEFT_OUTSIDE_POS = 0;
    public static final double AUTONOMOUS_CLAW_LEFT_WIDE_OPEN_POS = 0;
    public static final double AUTONOMOUS_CLAW_LEFT_ALL_CLOSED_POS = 0;
    public static final double AUTONOMOUS_CLAW_LEFT_GRAB_STONE_POS = 0;

    public static final RobotPoint2D AUTONOMOUS_CLAW_RIGHT_POSITION_WHEN_DOWN = new RobotPoint2D(0,0);
    public static final double AUTONOMOUS_CLAW_RIGHT_INSIDE_POS = 0;
    public static final double AUTONOMOUS_CLAW_RIGHT_REST_POS = 0;
    public static final double AUTONOMOUS_CLAW_RIGHT_OUTSIDE_POS = 0;
    public static final double AUTONOMOUS_CLAW_RIGHT_WIDE_OPEN_POS = 0;
    public static final double AUTONOMOUS_CLAW_RIGHT_ALL_CLOSED_POS = 0;
    public static final double AUTONOMOUS_CLAW_RIGHT_GRAB_STONE_POS = 0;

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
    public static final double OUTTAKE_SLIDE_MIN_POS = 0.0;
    public static final double OUTTAKE_SLIDE_MAX_POS = 0.0;
    public static final double OUTTAKE_SLIDE_OUT_OF_WAY_INTAKE_POS = 0.0;
    public static final ServoType OUTTAKE_SLIDE_SERVO_TYPE = new HS755MG();
    public static final double OUTTAKE_GRABBER_FAR_RELEASED_POS = 0.0;
    public static final double OUTTAKE_GRABBER_GRABBED_POS = 0.0;
    public static final double OUTTAKE_GRABBER_RELEASED_POS = 0.0;
    public static final ServoType OUTTAKE_GRABBER_SERVO_TYPE = new HS755HB();
    //========== End of Our-Take System Settings ==========

    //========== Start of Intake System Settings ==========
    public static final double INTAKE_POSITIONING_SERVO_REST = 0.0;
    public static final double INTAKE_POSITIONING_SERVO_HIT = 0.0;
    public static final ServoType INTAKE_POSITIONING_SERVO_TYPE = new HS755HB();
    //========== End of Intake System Settings ==========

    //========== Start of Capstone Delivery System Settings ==========
    public static final double CAPSTONE_SLIDE_MIN = 0;
    public static final double CAPSTONE_SLIDE_MAX = 0;
    public static final MotorType CAPSTONE_SLIDE_MOTOR_TYPE = new RevHDHex40Motor();

    public static final double CAPSTONE_SERVO_IN_POS = 0;
    public static final double CAPSTONE_SERVO_OUT_POS = 0;
    public static final ServoType CAPSTONE_SERVO_TYPE = new HS765HB();
    //========== End of Capstone Delivery System Settings ==========
}
