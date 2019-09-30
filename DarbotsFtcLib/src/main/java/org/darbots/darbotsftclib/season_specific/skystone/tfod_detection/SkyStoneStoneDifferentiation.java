package org.darbots.darbotsftclib.season_specific.skystone.tfod_detection;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotCamera;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

public class SkyStoneStoneDifferentiation {
    public enum StoneType{
        STONE,
        SKYSTONE
    }
    public static class RecognitionResult{
        private StoneType m_StoneType;
        private float m_Left;
        private float m_Top;
        private float m_Width;
        private float m_Height;
        private float m_ImageWidth;
        private float m_ImageHeight;
        private float m_Confidence;

        public RecognitionResult(StoneType stoneType, float Left, float Top, float Width, float Height, float ImageWidth, float ImageHeight, float Confidence){
            this.m_StoneType = stoneType;
            this.m_Left = Left;
            this.m_Top = Top;
            this.m_Width = Width;
            this.m_Height = Height;
            this.m_ImageWidth = ImageWidth;
            this.m_ImageHeight = ImageHeight;
            this.m_Confidence = Confidence;
        }
        public RecognitionResult(RecognitionResult otherResult){
            this.m_StoneType = otherResult.m_StoneType;
            this.m_Left = otherResult.m_Left;
            this.m_Top = otherResult.m_Top;
            this.m_Width = otherResult.m_Width;
            this.m_Height = otherResult.m_Height;
            this.m_ImageWidth = otherResult.m_ImageWidth;
            this.m_ImageHeight = otherResult.m_ImageHeight;
            this.m_Confidence = otherResult.m_Confidence;
        }
        public RecognitionResult(Recognition TFODRecognition){
            this.m_StoneType = TFODRecognition.getLabel().equals("Stone") ? StoneType.STONE : StoneType.SKYSTONE;
            this.m_Left = TFODRecognition.getLeft();
            this.m_Top = TFODRecognition.getTop();
            this.m_Width = TFODRecognition.getWidth();
            this.m_Height = TFODRecognition.getHeight();
            this.m_ImageWidth = TFODRecognition.getImageWidth();
            this.m_ImageHeight = TFODRecognition.getImageHeight();
            this.m_Confidence = TFODRecognition.getConfidence();
        }

        public StoneType getStoneType(){
            return this.m_StoneType;
        }
        public void setStoneType(StoneType stoneType){
            this.m_StoneType = stoneType;
        }
        public float getLeft(){
            return this.m_Left;
        }
        public void setLeft(float left){
            this.m_Left = left;
        }
        public float getTop(){
            return this.m_Top;
        }
        public void setTop(float top){
            this.m_Top = top;
        }
        public float getWidth(){
            return this.m_Width;
        }

        public void setWidth(float width){
            this.m_Width = width;
        }

        public float getHeight(){
            return this.m_Height;
        }
        public void setHeight(float height){
            this.m_Height = height;
        }
        public float getRight(){
            return this.m_Left + this.m_Width;
        }
        public float getBottom(){
            return this.m_Top + this.m_Height;
        }
        public float getImageWidth(){
            return this.m_ImageWidth;
        }
        public void setImageWidth(float ImageWidth){
            this.m_ImageWidth = ImageWidth;
        }
        public float getImageHeight(){
            return this.m_ImageHeight;
        }
        public void setImageHeight(float ImageHeight){
            this.m_ImageWidth = ImageHeight;
        }
        public float getConfidence(){
            return this.m_Confidence;
        }
        public void setConfidence(float Confidence){
            this.m_Confidence = Confidence;
        }

    }
    private RobotCamera m_Camera;
    private TFObjectDetector m_TFOD;
    private HardwareMap m_HardwareMap;
    private boolean m_Preview;
    public SkyStoneStoneDifferentiation(RobotCamera Camera, HardwareMap HardwareList, boolean Preview) throws Exception {
        this.m_Camera = Camera;
        this.m_HardwareMap = HardwareList;
        this.m_Preview = Preview;

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            __initTfod();
        } else {
            throw new Exception("Your device cannot create TFOD instances");
        }
    }

    public void terminate(){
        m_TFOD.shutdown();
    }

    public ArrayList<RecognitionResult> getUpdatedRecognitions(){
        List<Recognition> updatedRecognitions = m_TFOD.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            ArrayList<RecognitionResult> ResultArray = new ArrayList<RecognitionResult>();
            // step through the list of recognitions and display boundary info.
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
                ResultArray.add(new RecognitionResult(recognition));
            }
            return ResultArray;
        }else{
            return null;
        }
    }

    public RobotCamera getCamera(){
        return this.m_Camera;
    }
    public boolean isPreview(){
        return this.m_Preview;
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void __initTfod() {

        int tfodMonitorViewId = 0;
        TFObjectDetector.Parameters tfodParameters = null;
        if(m_Preview) {
            m_HardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", m_HardwareMap.appContext.getPackageName());
            tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        }else{
            tfodParameters = new TFObjectDetector.Parameters();
        }
        tfodParameters.minimumConfidence = 0.8;
        m_TFOD = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, this.m_Camera.getVuforia());
        m_TFOD.loadModelFromAsset("Skystone.tflite", "Stone", "Skystone");
    }

    public void setActivated(boolean enabled){
        if(enabled){
            this.m_TFOD.activate();
        }else{
            this.m_TFOD.deactivate();
        }
    }
}
