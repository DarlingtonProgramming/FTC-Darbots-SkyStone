package org.darbots.darbotsftclib.libcore.templates.motor_related;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;

public interface RobotMotor extends RobotNonBlockingDevice {
    enum MovingType{
        toCount,
        withSpeed,
        reset
    }
    MotorType getMotorType();
    void setMotorType(@NonNull MotorType MotorType);
    int getCurrentCount();
    int getTargetCount();
    void setTargetCount(int Count);
    double getPower();
    void setPower(double Pwr);
    boolean isDirectionReversed();
    void setDirectionReversed(boolean isReversed);
    DcMotor.ZeroPowerBehavior getZeroPowerBehavior();
    void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior ZeroPwrBehavior);
    MovingType getCurrentMovingType();
    void setCurrentMovingType(MovingType movingType);
    String getMotorStatusString();
}
