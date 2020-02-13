package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.motortypes.GoBilda5202Series1150RPMMotor;
import org.darbots.darbotsftclib.libcore.motortypes.MotorTypeUtil;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;

public class ElysiumSettings {
    //========== Start of Chassis Settings ==========
    public static final MotorType CHASSIS_MOTOR_TYPE = MotorTypeUtil.applyGearRatio(
            new GoBilda5202Series1150RPMMotor(),
            2.0
    );
    public static final double CHASSIS_WHEEL_RADIUS = 5.08; //4 Inch Diameter
    public static final double CHASSIS_LENGTH = 33.6; //14 GOBILDA BIG HOLES 14 * 24mm
    public static final double CHASSIS_WIDTH = 33.0;
    public static final RobotVector2D CHASSIS_VECTORS = new RobotVector2D(
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
}
