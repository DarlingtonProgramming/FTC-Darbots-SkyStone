package org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.purepursuit.followers.PurePursuitPathFollower;
import org.darbots.darbotsftclib.libcore.purepursuit.utils.PurePursuitWayPoint;
import org.darbots.darbotsftclib.libcore.runtime.MovementUtil;
import org.darbots.darbotsftclib.libcore.templates.DarbotsComboKey;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.MotionSystemConstraints;
import org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.ComboKeys.AUTO_DropStoneToFoundationCombo;
import org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.ComboKeys.AUTO_StopSuckStonesCombo;

import java.util.ArrayList;

public abstract class LindelAutoBase extends DarbotsBasicOpMode<LindelCore> {
    private LindelCore m_Core;
    public boolean pointMirrored = false;
    public DarbotsComboKey stopSuckStonesCombo;
    public DarbotsComboKey dropStoneToFoundationCombo;
    public MotionSystemConstraints constraints;
    public double maxAutoSpeed;
    public double maxAutoAngularSpeed;
    public double purePursuitAngSpeed;

    @Override
    public LindelCore getRobotCore() {
        return this.m_Core;
    }

    @Override
    public void hardwareInitialize() {
        this.m_Core = new LindelCore(this.hardwareMap,"LindelAutonomous.log");
        this.stopSuckStonesCombo = new AUTO_StopSuckStonesCombo(this.m_Core);
        this.dropStoneToFoundationCombo = new AUTO_DropStoneToFoundationCombo(this.m_Core);
        constraints = this.getRobotCore().getChassis().getMotionSystemConstraints(LindelSettings.AUTO_MAX_ACCEL_NORMALIZED * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),0,LindelSettings.AUTO_MAX_ANGULAR_ACCEL_NORMALIZED * this.getRobotCore().getChassis().calculateMaxAngularSpeedInDegPerSec(),0);
        maxAutoSpeed = this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec() * LindelSettings.AUTO_MAX_SPEED_NORMALIZED;
        maxAutoAngularSpeed = this.getRobotCore().getChassis().calculateMaxAngularSpeedInDegPerSec() * LindelSettings.AUTO_MAX_ANGULAR_NORMALIZED;
        purePursuitAngSpeed = LindelSettings.AUTO_PURE_PURSUIT_ANGLE_SPEED_NORMALIZED * this.getRobotCore().getChassis().calculateMaxAngularSpeedInDegPerSec();
        MovementUtil.drivetrain_constraints = constraints;
        MovementUtil.resolution = 0.02;
        this.m_Core.getGrabberRotServo().getServo().setPosition(this.m_Core.getGrabberRotServo().getCurrentPosition());
        __init();
    }

    public abstract void __init();

    @Override
    public void hardwareDestroy() {
        this.m_Core.saveAll();
        __destroy();
    }

    public abstract void __destroy();

    public void startSuckStones(){
        this.m_Core.setGrabberServoToGrab(false);
        this.m_Core.setIntakeSystemStatus(LindelCore.IntakeSystemStatus.SUCK,1.0);
    }

    public void stopSuckStones(){
        if(!stopSuckStonesCombo.isBusy()){
            stopSuckStonesCombo.startCombo();
        }
    }

    public ArrayList<PurePursuitWayPoint> transferWorldPointsToPurePursuitPoints(ArrayList<RobotPoint2D> WorldPoints, RobotPose2D currentRobotPosition){
        ArrayList<PurePursuitWayPoint> purePursuitContainer = new ArrayList<>();
        for(int i = 0; i<WorldPoints.size(); i++){
            RobotPoint2D robotPursuitPoint = XYPlaneCalculations.getRelativePosition(currentRobotPosition,WorldPoints.get(i));
            PurePursuitWayPoint robotPurePursuitPoint = new PurePursuitWayPoint(robotPursuitPoint.X,robotPursuitPoint.Y);
            purePursuitContainer.add(robotPurePursuitPoint);
        }
        return purePursuitContainer;
    }

    public PurePursuitPathFollower getFollower(ArrayList<PurePursuitWayPoint> pursuitPoints, double followRadius, double normalizedSpeed){
        return new PurePursuitPathFollower(pursuitPoints,this.constraints.maximumLinearAcceleration,0,normalizedSpeed * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),0,this.purePursuitAngSpeed,followRadius,0);
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
        }
        return this.opModeIsActive();
    }

    public boolean waitForDrive_ChassisOnly(){
        while(this.opModeIsActive() && this.getRobotCore().getChassis().isBusy()){
            this.getRobotCore().getChassis().updateStatus();
        }
        return this.opModeIsActive();
    }

    @Override
    public boolean waitForDrive_WithTelemetry(){
        while(this.opModeIsActive() && this.getRobotCore().getChassis().isBusy()){
            this.updateStatus();
            updateTelemetry();
            this.telemetry.update();
        }
        return this.opModeIsActive();
    }

    public void updateTelemetry(){
        this.getRobotCore().updateTelemetry();
        __updateTelemetry();
    }

    public abstract void __updateTelemetry();
}
