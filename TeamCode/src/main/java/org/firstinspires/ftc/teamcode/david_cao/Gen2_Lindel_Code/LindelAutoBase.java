package org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.templates.DarbotsComboKey;
import org.firstinspires.ftc.teamcode.david_cao.Gen4_SwanSilver_Code.SwanSilverCore;

import java.util.EventListenerProxy;

public abstract class LindelAutoBase extends DarbotsBasicOpMode<LindelCore> {
    private LindelCore m_Core;
    public boolean pointMirrored = false;
    public DarbotsComboKey stopSuckStonesCombo;

    @Override
    public LindelCore getRobotCore() {
        return this.m_Core;
    }

    @Override
    public void hardwareInitialize() {
        this.m_Core = new LindelCore(this.hardwareMap,"LindelAutonomous.log");
        this.stopSuckStonesCombo = new DarbotsComboKey() {
            ElapsedTime time;
            @Override
            protected void __startCombo() {
                m_Core.setIntakeSystemStatus(LindelCore.IntakeSystemStatus.VOMIT,0.3);
                m_Core.setGrabberServoToGrab(false);
                time = new ElapsedTime();
            }

            @Override
            protected void __stopCombo() {
                m_Core.setIntakeSystemStatus(LindelCore.IntakeSystemStatus.STOP,0);
                m_Core.setGrabberServoToGrab(true);
                m_Core.setOrientServoToOrient(false);
                time = null;
            }

            @Override
            public void updateStatus() {
                if(!this.isBusy()){
                    return;
                }
                double ms = time.milliseconds();
                if(ms >= 200 && ms < 500){
                    m_Core.setIntakeSystemStatus(LindelCore.IntakeSystemStatus.SUCK,0.4);
                }else if(ms >= 500 && ms < 1000){
                    m_Core.setIntakeSystemStatus(LindelCore.IntakeSystemStatus.STOP,0);
                    m_Core.setOrientServoToOrient(true);
                }else if(ms >= 1000){
                    m_Core.setOrientServoToOrient(false);
                    m_Core.setGrabberServoToGrab(true);
                    this.stopCombo();
                }
            }
        };
    }

    @Override
    public void hardwareDestroy() {
        this.m_Core.saveAll();
    }

    public void startSuckStones(){
        this.m_Core.setGrabberServoToGrab(false);
        this.m_Core.setIntakeSystemStatus(LindelCore.IntakeSystemStatus.SUCK,0.4);
    }

    public void stopSuckStones(){
        if(!stopSuckStonesCombo.isBusy()){
            stopSuckStonesCombo.startCombo();
        }
    }

    public RobotPoint2D getWorldPoint(RobotPoint2D rawPoint){
        if(!pointMirrored){
            return rawPoint;
        }else{
            RobotPoint2D newPoint = new RobotPoint2D(rawPoint.X, -rawPoint.Y);
            return newPoint;
        }
    }

    public RobotPose2D getWorldPose(RobotPose2D rawPose){
        if(!pointMirrored){
            return rawPose;
        }else{
            RobotPose2D newPose = new RobotPose2D(rawPose.X, -rawPose.Y, -rawPose.getRotationZ());
            return newPose;
        }
    }

    public double getWorldRotation(double rawRotation){
        if(!pointMirrored){
            return rawRotation;
        }else{
            return XYPlaneCalculations.normalizeDeg(-rawRotation);
        }
    }

    public RobotPose2D getRobotPose(RobotPose2D rawWorldPose){
        return this.getRobotCore().getChassis().getPositionTracker().robotAxisFromFieldAxis(this.getWorldPose(rawWorldPose));
    }

    public void updateStatus(){
        this.stopSuckStonesCombo.updateStatus();
        this.m_Core.updateStatus();
    }

    @Override
    public boolean waitForDrive(){
        while(this.opModeIsActive() && this.getRobotCore().getChassis().isBusy()){
            this.updateStatus();
            this.telemetry.update();
        }
        return this.opModeIsActive();
    }

    @Override
    public boolean waitForDrive_WithTelemetry(){
        while(this.opModeIsActive() && this.getRobotCore().getChassis().isBusy()){
            this.updateStatus();
            this.getRobotCore().updateTelemetry();
            this.telemetry.update();
        }
        return this.opModeIsActive();
    }
}
