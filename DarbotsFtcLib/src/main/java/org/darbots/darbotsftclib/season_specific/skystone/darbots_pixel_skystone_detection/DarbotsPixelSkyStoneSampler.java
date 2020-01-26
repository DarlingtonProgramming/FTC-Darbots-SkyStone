package org.darbots.darbotsftclib.season_specific.skystone.darbots_pixel_skystone_detection;

import android.graphics.Bitmap;
import android.graphics.Color;

import org.darbots.darbotsftclib.libcore_4_5_0Pre.integratedfunctions.image_processing.FTCImageUtility;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.templates.other_sensors.RobotCamera;
import org.darbots.darbotsftclib.season_specific.skystone.SkyStonePosition;
import org.darbots.darbotsftclib.season_specific.skystone.SkyStoneType;

public class DarbotsPixelSkyStoneSampler {
    public static final float BRIGHTNESS_SKYSTONE = 0.35f;
    public static final float HUE_MIN_STONE = 45.0f;
    public static final float HUE_MAX_STONE = 75.0f;

    private RobotCamera m_Camera;
    public DarbotsPixelSkyStoneSampler(RobotCamera camera){
        this.m_Camera = camera;
    }
    public RobotCamera getCamera(){
        return this.m_Camera;
    }
    public SkyStoneType getStoneType(int originalImageWidth, int originalImageHeight, int sampleImageWidth, int sampleImageHeight, int originalImageStartX, int originalImageStartY, int originalImageEndX, int originalImageEndY){
        int trial = 0;
        Bitmap frame = null;
        while(frame == null){
            trial++;
            if(trial > 3){
                return SkyStoneType.UNKNOWN;
            }
            frame = this.m_Camera.getFrame();
        }
        int countedColor = FTCImageUtility.countShrinkedScaledAverageColor(frame,sampleImageWidth,sampleImageHeight,originalImageWidth,originalImageHeight,originalImageStartX,originalImageStartY,originalImageEndX,originalImageEndY);
        float[] countedHSV = new float[3];
        Color.colorToHSV(countedColor,countedHSV);

        float hue = countedHSV[0];
        float saturation = countedHSV[1];
        float brightness = countedHSV[2];
        if(brightness <= BRIGHTNESS_SKYSTONE){
            return SkyStoneType.SKYSTONE;
        }
        if(hue >= HUE_MIN_STONE && hue <= HUE_MAX_STONE){
            return SkyStoneType.STONE;
        }
        return SkyStoneType.UNKNOWN;
    }
    public SkyStonePosition sample(int originalImageWidth, int originalImageHeight, int sampleImageWidth, int sampleImageHeight, int WallStoneStartX, int WallStoneStartY, int WallStoneEndX, int WallStoneEndY, int CenterStoneStartX, int CenterStoneStartY, int CenterStoneEndX, int CenterStoneEndY, int BridgeStoneStartX, int BridgeStoneStartY, int BridgeStoneEndX, int BridgeStoneEndY){
        int trial = 0;
        Bitmap frame = null;
        while(frame == null){
            trial++;
            if(trial > 3){
                return SkyStonePosition.UNKNOWN;
            }
            frame = this.m_Camera.getFrame();
        }
        Bitmap scaledBitmap = FTCImageUtility.getScaledImage(frame,sampleImageWidth,sampleImageHeight);
        int WallStoneColor = FTCImageUtility.countShrinkedScaledAverageColor(scaledBitmap,originalImageWidth,originalImageHeight,WallStoneStartX,WallStoneStartY,WallStoneEndX,WallStoneEndY);
        float[] WallStoneHSV = new float[3];
        Color.colorToHSV(WallStoneColor,WallStoneHSV);
        float WallStoneBrightness = WallStoneHSV[2];

        int CenterStoneColor = FTCImageUtility.countShrinkedScaledAverageColor(scaledBitmap,originalImageWidth,originalImageHeight,CenterStoneStartX,CenterStoneStartY,CenterStoneEndX,CenterStoneEndY);
        float[] CenterStoneHSV = new float[3];
        Color.colorToHSV(CenterStoneColor,CenterStoneHSV);
        float CenterStoneBrightness = CenterStoneHSV[2];

        int BridgeStoneColor = FTCImageUtility.countShrinkedScaledAverageColor(scaledBitmap,originalImageWidth,originalImageHeight,BridgeStoneStartX,BridgeStoneStartY,BridgeStoneEndX,BridgeStoneEndY);
        float[] BridgeStoneHSV = new float[3];
        Color.colorToHSV(BridgeStoneColor,BridgeStoneHSV);
        float BridgeStoneBrightness = BridgeStoneHSV[2];

        if(WallStoneBrightness <= CenterStoneBrightness && WallStoneBrightness <= BridgeStoneBrightness){
            return SkyStonePosition.NEXT_TO_WALL;
        }else if(CenterStoneBrightness <= WallStoneBrightness && CenterStoneBrightness <= BridgeStoneBrightness){
            return SkyStonePosition.MIDDLE;
        }else{
            return SkyStonePosition.NEXT_TO_BRIDGE;
        }
    }
}
