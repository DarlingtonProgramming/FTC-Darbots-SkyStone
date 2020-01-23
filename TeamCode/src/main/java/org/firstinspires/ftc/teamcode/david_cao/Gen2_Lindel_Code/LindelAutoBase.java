package org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.templates.DarbotsComboKey;
import org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.ComboKeys.AUTO_DropStoneToFoundationCombo;
import org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.ComboKeys.AUTO_StopSuckStonesCombo;

public abstract class LindelAutoBase extends DarbotsBasicOpMode<LindelCore> {
    private LindelCore m_Core;
    public boolean pointMirrored = false;
    public DarbotsComboKey stopSuckStonesCombo;
    public DarbotsComboKey dropStoneToFoundationCombo;

    @Override
    public LindelCore getRobotCore() {
        return this.m_Core;
    }

    @Override
    public void hardwareInitialize() {
        this.m_Core = new LindelCore(this.hardwareMap,"LindelAutonomous.log");
        this.stopSuckStonesCombo = new AUTO_StopSuckStonesCombo(this.m_Core);
        this.dropStoneToFoundationCombo = new AUTO_DropStoneToFoundationCombo(this.m_Core);
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

    public void depositStoneToFoundation(){
        if(!dropStoneToFoundationCombo.isBusy()) {
            this.dropStoneToFoundationCombo.startCombo();
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
        this.dropStoneToFoundationCombo.updateStatus();
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
