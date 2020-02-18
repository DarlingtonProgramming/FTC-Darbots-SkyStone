package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Autonomous;

import android.graphics.Bitmap;

import org.darbots.darbotsftclib.game_specific.AllianceType;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotCamera;
import org.darbots.darbotsftclib.season_specific.skystone.SkyStonePosition;
import org.darbots.darbotsftclib.season_specific.skystone.darbots_pixel_skystone_detection.DarbotsPixelSkyStoneSampler;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings.ElysiumAutonomousSettings;

public class ElysiumAutoSampler extends DarbotsPixelSkyStoneSampler {
    private Bitmap m_LastSampledFrame;
    public ElysiumAutoSampler(RobotCamera camera) {
        super(camera);
    }
    public SkyStonePosition sampleRed(){
        m_LastSampledFrame = super.getFrame();
        return super.sample(
                m_LastSampledFrame,
                true,
                ElysiumAutonomousSettings.SAMPLE_PICTURE_SIZE_X,
                ElysiumAutonomousSettings.SAMPLE_PICTURE_SIZE_Y,
                ElysiumAutonomousSettings.SAMPLE_SHRINKED_SIZE_X,
                ElysiumAutonomousSettings.SAMPLE_SHRINKED_SIZE_Y,
                ElysiumAutonomousSettings.SAMPLE_RED_STONE_NEXT_TO_WALL_START_X,
                ElysiumAutonomousSettings.SAMPLE_RED_STONE_NEXT_TO_WALL_START_Y,
                ElysiumAutonomousSettings.SAMPLE_RED_STONE_NEXT_TO_WALL_END_X,
                ElysiumAutonomousSettings.SAMPLE_RED_STONE_NEXT_TO_WALL_END_Y,
                ElysiumAutonomousSettings.SAMPLE_RED_STONE_MIDDLE_START_X,
                ElysiumAutonomousSettings.SAMPLE_RED_STONE_MIDDLE_START_Y,
                ElysiumAutonomousSettings.SAMPLE_RED_STONE_MIDDLE_END_X,
                ElysiumAutonomousSettings.SAMPLE_RED_STONE_MIDDLE_END_Y,
                ElysiumAutonomousSettings.SAMPLE_RED_STONE_NEXT_TO_BRIDGE_START_X,
                ElysiumAutonomousSettings.SAMPLE_RED_STONE_NEXT_TO_BRIDGE_START_Y,
                ElysiumAutonomousSettings.SAMPLE_RED_STONE_NEXT_TO_BRIDGE_END_X,
                ElysiumAutonomousSettings.SAMPLE_RED_STONE_NEXT_TO_BRIDGE_END_Y
        );
    }
    public SkyStonePosition sampleBlue(){
        m_LastSampledFrame = super.getFrame();
        return super.sample(
                m_LastSampledFrame,
                true,
                ElysiumAutonomousSettings.SAMPLE_PICTURE_SIZE_X,
                ElysiumAutonomousSettings.SAMPLE_PICTURE_SIZE_Y,
                ElysiumAutonomousSettings.SAMPLE_SHRINKED_SIZE_X,
                ElysiumAutonomousSettings.SAMPLE_SHRINKED_SIZE_Y,
                ElysiumAutonomousSettings.SAMPLE_BLUE_STONE_NEXT_TO_WALL_START_X,
                ElysiumAutonomousSettings.SAMPLE_BLUE_STONE_NEXT_TO_WALL_START_Y,
                ElysiumAutonomousSettings.SAMPLE_BLUE_STONE_NEXT_TO_WALL_END_X,
                ElysiumAutonomousSettings.SAMPLE_BLUE_STONE_NEXT_TO_WALL_END_Y,
                ElysiumAutonomousSettings.SAMPLE_BLUE_STONE_MIDDLE_START_X,
                ElysiumAutonomousSettings.SAMPLE_BLUE_STONE_MIDDLE_START_Y,
                ElysiumAutonomousSettings.SAMPLE_BLUE_STONE_MIDDLE_END_X,
                ElysiumAutonomousSettings.SAMPLE_BLUE_STONE_MIDDLE_END_Y,
                ElysiumAutonomousSettings.SAMPLE_BLUE_STONE_NEXT_TO_BRIDGE_START_X,
                ElysiumAutonomousSettings.SAMPLE_BLUE_STONE_NEXT_TO_BRIDGE_START_Y,
                ElysiumAutonomousSettings.SAMPLE_BLUE_STONE_NEXT_TO_BRIDGE_END_X,
                ElysiumAutonomousSettings.SAMPLE_BLUE_STONE_NEXT_TO_BRIDGE_END_Y
        );
    }
    public SkyStonePosition sample(AllianceType allianceType){
        if(allianceType == AllianceType.BLUE){
            return this.sampleBlue();
        }else{
            return this.sampleRed();
        }
    }
    public Bitmap getLastSampledFrame(){
        return this.m_LastSampledFrame;
    }
}
