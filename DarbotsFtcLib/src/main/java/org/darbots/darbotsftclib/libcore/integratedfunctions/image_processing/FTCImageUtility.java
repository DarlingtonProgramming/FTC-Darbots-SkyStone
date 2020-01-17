package org.darbots.darbotsftclib.libcore.integratedfunctions.image_processing;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.support.annotation.ColorInt;

import org.darbots.darbotsftclib.libcore.calculations.algebraic_calculation.AlgebraicCalculations;

public class FTCImageUtility {
    @ColorInt
    public static int countScaledAverageColor(Bitmap image, int originalWidth, int originalHeight, int originalStartX, int originalStartY, int originalEndX, int originalEndY){
        //rescale the bitmap
        Bitmap scaledBitmap = Bitmap.createScaledBitmap(image,originalWidth,originalHeight,true);
        return countAverageColor(scaledBitmap,originalStartX,originalEndY,originalEndX,originalEndY);
    }
    @ColorInt
    public static int countAverageColor(Bitmap image, int startX, int startY, int endX, int endY){
        //Crop Bitmap
        int imageWidth = endX - startX + 1;
        int imageHeight = endY - endY + 1;
        Bitmap croppedIMG = Bitmap.createBitmap(image,startX,startY,imageWidth,imageHeight);
        long RedChannelCounter = 0L, GreenChannelCounter = 0L, BlueChannelCounter = 0L, AlphaChannelCounter = 0L;
        int pixelNum = imageWidth * imageHeight;
        for(int x = 0; x < imageWidth; x++){
            for(int y = 0; y < imageHeight; y++){
                int pixelColor = croppedIMG.getPixel(x,y);
                int pixelR, pixelG, pixelB, pixelA;
                pixelR = Color.red(pixelColor);
                pixelG = Color.green(pixelColor);
                pixelB = Color.blue(pixelColor);
                pixelA = Color.alpha(pixelColor);
                RedChannelCounter += pixelR;
                GreenChannelCounter += pixelG;
                BlueChannelCounter += pixelB;
                AlphaChannelCounter += pixelA;
            }
        }
        int avgRed = (int) Math.round(((double) RedChannelCounter) / pixelNum);
        int avgGreen = (int) Math.round(((double) GreenChannelCounter) / pixelNum);
        int avgBlue = (int) Math.round(((double) BlueChannelCounter) / pixelNum);
        int avgAlpha = (int) Math.round(((double) AlphaChannelCounter) / pixelNum);
        int colorVal = Color.argb(avgAlpha,avgRed,avgGreen,avgBlue);
        return colorVal;
    }
    @ColorInt
    public static int countShrinkedScaledAverageColor(Bitmap image, int sampleWidth, int sampleHeight, int originalWidth, int originalHeight, int originalStartX, int originalStartY, int originalEndX, int originalEndY){
        //rescale the bitmap
        Bitmap scaledBitmap = Bitmap.createScaledBitmap(image,sampleWidth,sampleHeight,true);
        int sampleStartX = AlgebraicCalculations.map(originalStartX,0,originalWidth,0,sampleWidth);
        int sampleEndX = AlgebraicCalculations.map(originalEndX,0,originalWidth,0,sampleWidth);
        int sampleStartY = AlgebraicCalculations.map(originalStartY,0,originalHeight,0,sampleHeight);
        int sampleEndY = AlgebraicCalculations.map(originalEndY,0,originalHeight,0,sampleHeight);
        return countAverageColor(scaledBitmap,sampleStartX,sampleStartY,sampleEndX,sampleEndY);
    }
}
