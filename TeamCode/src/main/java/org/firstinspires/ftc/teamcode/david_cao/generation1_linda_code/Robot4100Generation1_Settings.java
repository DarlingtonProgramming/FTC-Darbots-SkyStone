package org.firstinspires.ftc.teamcode.david_cao.generation1_linda_code;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.RobotPose3D;
import org.darbots.darbotsftclib.libcore.motortypes.AndyMark2964;
import org.darbots.darbotsftclib.libcore.motortypes.AndyMark3637;
import org.darbots.darbotsftclib.libcore.motortypes.servotypes.HS755HB;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;
import org.darbots.darbotsftclib.libcore.templates.servo_related.ServoType;

public class Robot4100Generation1_Settings {
    //------------Autonomous Settings------------
    public static final double AUTONOMOUS_MINIMUM_CONFIDENCE = 0.5;
    public static final double AUTONOMOUS_LENGTH_FOR_EACH_STONE = 20;
    public static final boolean AUTONOMOUS_TENSORFLOW_PREVIEW = true;
    public static final double AUTONOMOUS_DISTANCE_BETWEEN_PHONE_AND_STONEGRABBER = -10.5;
    public static final RobotPose3D AUTONOMOUS_CAMERAPOSONPHONE = new RobotPose3D(
            0,
            13.2,
            0,
            0,
            90,
            -90
    );
    //------------End of Autonomous Settings------------

    //------------TeleOp Settings------------
    public static final double TELEOP_MAXSPEED = 0.6;
    public static final double TELEOP_LINEARSLIDESPEED = 0.8;
    public static final double TELEOP_GRABBERROT_IN_SPEED = 1.0;
    public static final double TELEOP_GRABBERROT_OUT_SPEED = 0.5;

    //------------End of TeleOp Settings------------
    //------------Configuration Settings------------
    public static final double[] wheelPosition = {18.415,14};


    public static final MotorType motorType = new AndyMark3637();
    public static final double wheelRadius = 5;
    public static final MotorType linearSlideMotorType = new AndyMark2964();

    public static final boolean LINEARSLIDE_TIMEOUTCONTROLENABLE = true;
    public static final double LINEARSLIDE_TIMEOUTFACTOR = 1.5;
    public static final double LINEARSLIDE_MAX = 7.2804;
    public static final double LINEARSLIDE_MIN = 0;
    public static final double LINEARSLIDE_GRAB = 0.88843;
    public static final double LINEARSLIDE_START = 0;

    public static final double INTAKEMOTOR_SPEED = 0.8;


    public static final boolean CHASSIS_TIMEOUTENABLE = false;
    public static final double CHASSIS_TIMEOUTFACTOR = 1.0;
    public static final double DRAGSERVO_RESTPOS_L = 0.0;
    public static final double DRAGSERVO_RESTPOS_R = 1.0;
    public static final double DRAGSERVO_DRAGPOS_L = 1.0;
    public static final double DRAGSERVO_DRAGPOS_R = 0;
    public static final double GRABBERSERVO_RESTPOS = 0.4;
    public static final double GRABBERSERVO_GRABPOS = 0.1;
    public static final double GRABBERROTSERVO_INSIDEPOS = 0.97;
    public static final double GRABBERROTSERVO_OUTSIDEPOS = 0.025;
    public static final ServoType GRABBERROTSERVO_TYPE = new HS755HB();

    public static final double STONEORIENTSERVO_ZEROPOS = 0;
    public static final double STONEORIENTSERVO_ORIENTPOS = 0.5;

    public static final double AUTONOMOUSDRAGSTONESERVO_LEFT_OUTPOS = 0;
    public static final double AUTONOMOUSDRAGSTONESERVO_LEFT_INPOS = 0.6;

    public static final double AUTONOMOUSDRAGSTONESERVO_RIGHT_OUTPOS = 1.0;
    public static final double AUTONOMOUSDRAGSTONESERVO_RIGHT_IN = 0.4;

    public static final double CAPSTONESERVO_INITIALPOS = 1.0;
    public static final double CAPSTONESERVO_DEPOSITPOS = 0.0;
    //------------End of Configuration Settings------------

}
