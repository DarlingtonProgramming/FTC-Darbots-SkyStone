package org.darbots.darbotsftclib.libcore.sensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DarbotsDistanceSensor implements RobotNonBlockingDevice {
    public static double DISTANCE_INVALID = -1;

    public double ActualDistanceFactor = 1.0;
    private DistanceSensor m_DistanceSensor;
    private double m_LastReadingCM = DISTANCE_INVALID;

    public DarbotsDistanceSensor(DistanceSensor distanceSensor){
        this.m_DistanceSensor = distanceSensor;
    }

    public DarbotsDistanceSensor(DarbotsDistanceSensor sensor){
        this.m_DistanceSensor = sensor.m_DistanceSensor;
        this.m_LastReadingCM = sensor.m_LastReadingCM;
    }

    public DistanceSensor getDistanceSensor(){
        return this.m_DistanceSensor;
    }

    public void setDistanceSensor(DistanceSensor distanceSensor){
        this.m_DistanceSensor = distanceSensor;
    }

    public double getDistanceInCM(){
        return this.m_LastReadingCM;
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public void updateStatus() {
        this.m_LastReadingCM = this.m_DistanceSensor.getDistance(DistanceUnit.CM);
        if(this.m_LastReadingCM == DistanceSensor.distanceOutOfRange){
            this.m_LastReadingCM = DISTANCE_INVALID;
        }else{
            this.m_LastReadingCM *= this.ActualDistanceFactor;
        }
    }

    @Override
    public void waitUntilFinish() {
        return;
    }
}
