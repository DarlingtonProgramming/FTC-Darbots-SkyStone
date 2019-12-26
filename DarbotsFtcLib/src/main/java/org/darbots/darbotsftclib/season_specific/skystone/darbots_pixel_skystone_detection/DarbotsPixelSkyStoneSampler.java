package org.darbots.darbotsftclib.season_specific.skystone.darbots_pixel_skystone_detection;

import android.graphics.Bitmap;
import android.graphics.Color;

import org.darbots.darbotsftclib.libcore.integratedfunctions.image_processing.FTCImageUtility;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotCamera;
import org.darbots.darbotsftclib.season_specific.skystone.SkyStoneType;

public class DarbotsPixelSkyStoneSampler {
    public static final float BRIGHTNESS_SKYSTONE = 0.2f;
    public static final float HUE_MIN_STONE = 45.0f;
    public static final float HUE_MAX_STONE = 75.0f;

    private RobotCamera m_Camera;
    public DarbotsPixelSkyStoneSampler(RobotCamera camera){
        this.m_Camera = camera;
    }
    public RobotCamera getCamera(){
        return this.m_Camera;
    }
    public SkyStoneType getStoneType(int originalImageWidth, int originalImageHeight, int originalImageStartX, int originalImageStartY, int originalImageEndX, int originalImageEndY){
        int trial = 0;
        Bitmap frame = null;
        while(frame == null){
            frame = this.m_Camera.getFrame();
            trial++;
            if(trial > 3){
                return SkyStoneType.UNKNOWN;
            }
        }
        int countedColor = FTCImageUtility.countScaledAverageColor(frame,originalImageWidth,originalImageHeight,originalImageStartX,originalImageStartY,originalImageEndX,originalImageEndY);
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
}
