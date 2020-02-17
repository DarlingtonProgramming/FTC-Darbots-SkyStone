package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.integratedfunctions.FTCMemory;
import org.darbots.darbotsftclib.libcore.templates.odometry.CustomizableOdometry;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.RoadRunner.drive.localizer.RoadRunnerLocalizer;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.RoadRunner.drive.mecanum.ElysiumRoadRunnerChassis;

public class ElysiumAutoCore extends ElysiumCore {
    public ElysiumRoadRunnerChassis chassis;
    private CustomizableOdometry oldPosTracker;

    public ElysiumAutoCore(String logFileName, HardwareMap hardwareMap, boolean read, RobotPose2D initialPose, boolean distanceEnhancedLocalization) {
        super(logFileName, hardwareMap, read, initialPose, distanceEnhancedLocalization);
        //this.getChassis().terminate();
        oldPosTracker = (CustomizableOdometry) this.m_Chassis.getPositionTracker();
        this.m_Chassis = null;
        this.chassis = new ElysiumRoadRunnerChassis(hardwareMap);
        //this.chassis.setLocalizer(new RoadRunnerLocalizer(oldPosTracker));
        this.setCurrentPosition(initialPose);
    }
    public void save(){
        this.capstoneDeliverySubSystem.save();
        this.outtakeSubSystem.save();
        this.stackerSubSystem.save();
        RobotPose2D currentPosition = this.getCurrentPosition();
        FTCMemory.setSetting("ElysiumRobotPose",currentPosition);
        FTCMemory.saveMemoryMapToFile();
    }
    public void read(RobotPose2D defaultPose){
        this.capstoneDeliverySubSystem.read();
        this.outtakeSubSystem.read();
        this.stackerSubSystem.read();
        try {
            RobotPose2D readPosition = FTCMemory.getSetting("ElysiumRobotPose", defaultPose);
            this.setCurrentPosition(readPosition);
        }catch(Exception e){
            //do nothing
        }
    }
    @Override
    protected void __stop() {
        this.stackerSubSystem.stop();
        this.intakeSubSystem.stop();
        this.outtakeSubSystem.stop();
        this.capstoneDeliverySubSystem.stop();
    }
    @Override
    protected void __terminate() {
        this.oldPosTracker.stop();
    }

    @Override
    protected void __updateStatus() {
        this.stackerSubSystem.updateStatus();
        this.outtakeSubSystem.updateStatus();
        this.intakeSubSystem.updateStatus();
        this.capstoneDeliverySubSystem.updateStatus();
    }

    @Override
    public void __updateTelemetry(Telemetry telemetry, TelemetryPacket telemetryPacket) {
        this.capstoneDeliverySubSystem.updateTelemetry(telemetry,telemetryPacket);
        this.intakeSubSystem.updateTelemetry(telemetry,telemetryPacket);
        this.outtakeSubSystem.updateTelemetry(telemetry,telemetryPacket);
        this.stackerSubSystem.updateTelemetry(telemetry,telemetryPacket);
    }

    public RobotPose2D getCurrentPosition(){
        Pose2d currentEstimate = this.chassis.getPoseEstimate();
        return new RobotPose2D(currentEstimate.getX() / XYPlaneCalculations.INCH_PER_CM, currentEstimate.getY() / XYPlaneCalculations.INCH_PER_CM,Math.toDegrees(currentEstimate.getHeading()));
    }

    public void setCurrentPosition(RobotPose2D position){
        this.chassis.setPoseEstimate(new Pose2d(position.X * XYPlaneCalculations.INCH_PER_CM,position.Y * XYPlaneCalculations.INCH_PER_CM,Math.toRadians(position.getRotationZ())));
    }
}
