package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings;

import com.acmerobotics.dashboard.config.Config;

import org.darbots.darbotsftclib.game_specific.AllianceType;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.season_specific.skystone.ParkPosition;
import org.darbots.darbotsftclib.season_specific.skystone.SkyStoneCoordinates;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Autonomous.ElysiumAutoBase;

@Config
public class ElysiumAutonomousSettings {

    //========== Start of Sampling Settings ==========
    public static boolean SAMPLE_PREVIEW = true;
    
    public static int SAMPLE_RED_STONE_NEXT_TO_BRIDGE_START_X = 805;
    public static int SAMPLE_RED_STONE_NEXT_TO_BRIDGE_START_Y = 615;
    public static int SAMPLE_RED_STONE_NEXT_TO_BRIDGE_END_X = 995;
    public static int SAMPLE_RED_STONE_NEXT_TO_BRIDGE_END_Y = 690;
    public static int SAMPLE_RED_STONE_MIDDLE_START_X = 552;
    public static int SAMPLE_RED_STONE_MIDDLE_START_Y = 595;
    public static int SAMPLE_RED_STONE_MIDDLE_END_X = 745;
    public static int SAMPLE_RED_STONE_MIDDLE_END_Y = 685;
    public static int SAMPLE_RED_STONE_NEXT_TO_WALL_START_X = 294;
    public static int SAMPLE_RED_STONE_NEXT_TO_WALL_START_Y = 594;
    public static int SAMPLE_RED_STONE_NEXT_TO_WALL_END_X = 496;
    public static int SAMPLE_RED_STONE_NEXT_TO_WALL_END_Y = 674;

    public static int SAMPLE_BLUE_STONE_NEXT_TO_BRIDGE_START_X = 998;
    public static int SAMPLE_BLUE_STONE_NEXT_TO_BRIDGE_START_Y = 536;
    public static int SAMPLE_BLUE_STONE_NEXT_TO_BRIDGE_END_X = 1203;
    public static int SAMPLE_BLUE_STONE_NEXT_TO_BRIDGE_END_Y = 627;
    public static int SAMPLE_BLUE_STONE_MIDDLE_START_X = 754;
    public static int SAMPLE_BLUE_STONE_MIDDLE_START_Y = 527;
    public static int SAMPLE_BLUE_STONE_MIDDLE_END_X = 933;
    public static int SAMPLE_BLUE_STONE_MIDDLE_END_Y = 626;
    public static int SAMPLE_BLUE_STONE_NEXT_TO_WALL_START_X = 504;
    public static int SAMPLE_BLUE_STONE_NEXT_TO_WALL_START_Y = 526;
    public static int SAMPLE_BLUE_STONE_NEXT_TO_WALL_END_X = 675;
    public static int SAMPLE_BLUE_STONE_NEXT_TO_WALL_END_Y = 619;

    public static int SAMPLE_PICTURE_SIZE_X = 1280;
    public static int SAMPLE_PICTURE_SIZE_Y = 720;

    public static int SAMPLE_SHRINKED_SIZE_X = 320;
    public static int SAMPLE_SHRINKED_SIZE_Y = 180;
    //========== End of Sampling Settings ==========

    public static RobotPose2D RED_AUTO_START_POSE = new RobotPose2D(
            SkyStoneCoordinates.RED_LOADING_ZONE_FIELD_EXTREME_POINT.X + (120.2 - (ElysiumSettings.CHASSIS_LENGTH / 2.0 + ElysiumSettings.CHASSIS_WHEEL_RADIUS)),
            SkyStoneCoordinates.RED_LOADING_ZONE_FIELD_EXTREME_POINT.Y + ElysiumSettings.PHYSICAL_CENTER_TO_RIGHT_DOOR,
            0
    );
    public static RobotPose2D BLUE_AUTO_START_POSE = new RobotPose2D(
            SkyStoneCoordinates.BLUE_LOADING_ZONE_FIELD_EXTREME_POINT.X + (120.2 - ElysiumSettings.PHYSICAL_CENTER_TO_BACK),
            SkyStoneCoordinates.BLUE_BUILDING_ZONE_FIELD_EXTREME_POINT.Y - ElysiumSettings.PHYSICAL_CENTER_TO_RIGHT_DOOR,
            -180
    );

    public static double STACKER_SLIDE_SPEED = 1.0;

    public static double PURE_PURSUIT_ANGLE_SPEED = 0.15;

    public static boolean DISTANCE_SENSOR_POSITION_CALIBRATION = false;

    public static double DISTANCE_SENSOR_CALIBRATION_STARTX = ElysiumAutoBase.getLoadingZoneFurtherFromBridgePoint(AllianceType.BLUE,ParkPosition.NEXT_TO_WALL).X;
    public static double DISTANCE_SENSOR_CALIBRATION_ENDX = ElysiumAutoBase.getBuildingZoneFurtherFromBridgePoint(AllianceType.BLUE,ParkPosition.NEXT_TO_WALL).X;
}
