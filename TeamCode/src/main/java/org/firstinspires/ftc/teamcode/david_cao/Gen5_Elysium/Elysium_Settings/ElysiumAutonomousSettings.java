package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.season_specific.skystone.SkyStoneCoordinates;

public class ElysiumAutonomousSettings {

    //========== Start of Sampling Settings ==========
    public static final int SAMPLE_RED_STONE_NEXT_TO_BRIDGE_START_X = 880;
    public static final int SAMPLE_RED_STONE_NEXT_TO_BRIDGE_START_Y = 525;
    public static final int SAMPLE_RED_STONE_NEXT_TO_BRIDGE_END_X = 1063;
    public static final int SAMPLE_RED_STONE_NEXT_TO_BRIDGE_END_Y = 622;
    public static final int SAMPLE_RED_STONE_MIDDLE_START_X = 620;
    public static final int SAMPLE_RED_STONE_MIDDLE_START_Y = 520;
    public static final int SAMPLE_RED_STONE_MIDDLE_END_X = 807;
    public static final int SAMPLE_RED_STONE_MIDDLE_END_Y = 616;
    public static final int SAMPLE_RED_STONE_NEXT_TO_WALL_START_X = 362;
    public static final int SAMPLE_RED_STONE_NEXT_TO_WALL_START_Y = 525;
    public static final int SAMPLE_RED_STONE_NEXT_TO_WALL_END_X = 573;
    public static final int SAMPLE_RED_STONE_NEXT_TO_WALL_END_Y = 612;

    public static final int SAMPLE_BLUE_STONE_NEXT_TO_BRIDGE_START_X = 998;
    public static final int SAMPLE_BLUE_STONE_NEXT_TO_BRIDGE_START_Y = 536;
    public static final int SAMPLE_BLUE_STONE_NEXT_TO_BRIDGE_END_X = 1203;
    public static final int SAMPLE_BLUE_STONE_NEXT_TO_BRIDGE_END_Y = 627;
    public static final int SAMPLE_BLUE_STONE_MIDDLE_START_X = 754;
    public static final int SAMPLE_BLUE_STONE_MIDDLE_START_Y = 527;
    public static final int SAMPLE_BLUE_STONE_MIDDLE_END_X = 933;
    public static final int SAMPLE_BLUE_STONE_MIDDLE_END_Y = 626;
    public static final int SAMPLE_BLUE_STONE_NEXT_TO_WALL_START_X = 504;
    public static final int SAMPLE_BLUE_STONE_NEXT_TO_WALL_START_Y = 526;
    public static final int SAMPLE_BLUE_STONE_NEXT_TO_WALL_END_X = 675;
    public static final int SAMPLE_BLUE_STONE_NEXT_TO_WALL_END_Y = 619;

    public static final int SAMPLE_PICTURE_SIZE_X = 1280;
    public static final int SAMPLE_PICTURE_SIZE_Y = 720;

    public static final int SAMPLE_SHRINKED_SIZE_X = 320;
    public static final int SAMPLE_SHRINKED_SIZE_Y = 180;
    //========== End of Sampling Settings ==========

    public static final RobotPose2D RED_AUTO_START_POSE = new RobotPose2D(
            SkyStoneCoordinates.RED_LOADING_ZONE_FIELD_EXTREME_POINT.X + (120.2 - (ElysiumSettings.CHASSIS_LENGTH / 2.0 + ElysiumSettings.CHASSIS_WHEEL_RADIUS)),
            SkyStoneCoordinates.RED_LOADING_ZONE_FIELD_EXTREME_POINT.Y + ElysiumSettings.PHYSICAL_CENTER_TO_RIGHT_DOOR,
            0
    );
    public static final RobotPose2D BLUE_AUTO_START_POSE = new RobotPose2D(
            SkyStoneCoordinates.BLUE_LOADING_ZONE_FIELD_EXTREME_POINT.X + (120.2 - ElysiumSettings.PHYSICAL_CENTER_TO_BACK),
            SkyStoneCoordinates.BLUE_BUILDING_ZONE_FIELD_EXTREME_POINT.Y - ElysiumSettings.PHYSICAL_CENTER_TO_RIGHT_DOOR,
            -180
    );

}
