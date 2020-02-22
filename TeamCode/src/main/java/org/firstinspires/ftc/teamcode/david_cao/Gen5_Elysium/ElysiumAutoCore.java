package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.integratedfunctions.DarbotsOnRobotSensor2D;
import org.darbots.darbotsftclib.libcore.integratedfunctions.FTCMemory;
import org.darbots.darbotsftclib.libcore.motionsystems.MecanumDrivetrain;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.sensors.distance_sensors.DarbotsRevDistanceSensor;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;
import org.darbots.darbotsftclib.libcore.templates.odometry.RobotAsyncPositionTracker;
import org.darbots.darbotsftclib.libcore.templates.sensors.DarbotsDistanceSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings.ElysiumSettings;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.RoadRunner.drive.localizer.RoadRunnerLocalizer;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.RoadRunner.drive.localizer.RoadRunnerMecanumOdometry;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.RoadRunner.drive.mecanum.ElysiumRoadRunnerChassis;

public class ElysiumAutoCore extends ElysiumCore {
    public ElysiumRoadRunnerChassis chassis;
    public DarbotsOnRobotSensor2D<DarbotsDistanceSensor> FrontSensor, LeftSensor, BackSensor, RightSensor;
    public static String robotDrawColor = "#000066";
    public RobotMotionSystem oldMotionSystem;

    public ElysiumAutoCore(String logFileName, HardwareMap hardwareMap, boolean read, RobotPose2D initialPose, boolean distanceEnhancedLocalization) {
        super(logFileName, hardwareMap, false, true, initialPose, distanceEnhancedLocalization);
        this.chassis = new ElysiumRoadRunnerChassis(hardwareMap);

        this.oldMotionSystem = this.getChassis();
        this.oldMotionSystem.getPositionTracker().stop();
        this.m_Chassis = null;
        RoadRunnerMecanumOdometry odometryMethod = new RoadRunnerMecanumOdometry((MecanumDrivetrain) this.oldMotionSystem);
        RobotAsyncPositionTracker positionTracker = new RobotAsyncPositionTracker(odometryMethod,initialPose);
        positionTracker.setDistanceFactors(ElysiumSettings.CHASSIS_FACTORS);
        positionTracker.setGyroProvider(this.getGyro());
        positionTracker.start();

        //this.chassis.setLocalizer(new RoadRunnerLocalizer((RobotAsyncPositionTracker) this.oldMotionSystem.getPositionTracker()));
        this.setCurrentPosition(initialPose);
        this.m_Chassis = null;
        FrontSensor = new DarbotsOnRobotSensor2D<DarbotsDistanceSensor>(ElysiumSettings.LOCALIZATION_FRONTDISTSENSOR_POS,new DarbotsRevDistanceSensor(hardwareMap.get(DistanceSensor.class,"frontDistanceSensor")));
        FrontSensor.Sensor.ActualDistanceFactor = ElysiumSettings.LOCALIZATION_FRONTDISTANCESENSOR_FACTOR;

        LeftSensor = new DarbotsOnRobotSensor2D<DarbotsDistanceSensor>(ElysiumSettings.LOCALIZATION_LEFTDISTSENSOR_POS,new DarbotsRevDistanceSensor(hardwareMap.get(DistanceSensor.class,"leftDistanceSensor")));
        LeftSensor.Sensor.ActualDistanceFactor = ElysiumSettings.LOCALIZATION_LEFTDISTANCESENSOR_FACTOR;

        BackSensor = new DarbotsOnRobotSensor2D<DarbotsDistanceSensor>(ElysiumSettings.LOCALIZATION_BACKDISTSENSOR_POS,new DarbotsRevDistanceSensor(hardwareMap.get(DistanceSensor.class,"backDistanceSensor")));
        BackSensor.Sensor.ActualDistanceFactor = ElysiumSettings.LOCALIZATION_BACKDISTANCESENSOR_FACTOR;

        RightSensor = new DarbotsOnRobotSensor2D<DarbotsDistanceSensor>(ElysiumSettings.LOCALIZATION_RIGHTDISTSENSOR_POS,new DarbotsRevDistanceSensor(hardwareMap.get(DistanceSensor.class,"rightDistanceSensor")));
        RightSensor.Sensor.ActualDistanceFactor = ElysiumSettings.LOCALIZATION_RIGHTDISTANCESENSOR_FACTOR;
    }
    public void save(){
        super.save();
        RobotPose2D currentPosition = this.getCurrentPosition();
        FTCMemory.setSetting("ElysiumRobotPose",currentPosition);
        FTCMemory.saveMemoryMapToFile();
    }
    public void read(RobotPose2D defaultPose){
        super.read(defaultPose);
        try {
            RobotPose2D readPosition = FTCMemory.getSetting("ElysiumRobotPose", defaultPose);
            this.setCurrentPosition(readPosition);
        }catch(Exception e){
            //do nothing
        }
    }
    @Override
    protected void __stop() {
        super.__stop();
    }
    @Override
    protected void __terminate() {
        this.oldMotionSystem.terminate();
        super.__terminate();
    }

    @Override
    protected void __updateStatus() {
        super.__updateStatus();
        ((RobotNonBlockingDevice) this.oldMotionSystem.getPositionTracker()).updateStatus();
    }

    @Override
    public void __updateTelemetry(Telemetry telemetry, TelemetryPacket telemetryPacket) {
        GlobalUtil.addTelmetryLine(telemetry,telemetryPacket,"position",this.getCurrentPosition().toString());
        this.__drawFieldOverlay(telemetryPacket.fieldOverlay());
        super.__updateTelemetry(telemetry,telemetryPacket);
    }

    public void __drawFieldOverlay(Canvas canvas){
        RobotPose2D currentRobotPosition = this.getCurrentPosition();
        RobotPoint2D[] robotAxisPoints = XYPlaneCalculations.getRobotExtremeBoundingBox(ElysiumSettings.PHYSICAL_CENTER_TO_FRONT,ElysiumSettings.PHYSICAL_CENTER_TO_BACK,ElysiumSettings.PHYSICAL_CENTER_TO_LEFT,ElysiumSettings.PHYSICAL_CENTER_TO_RIGHT);
        RobotPoint2D tempRBPoint = robotAxisPoints[3];
        robotAxisPoints[3] = robotAxisPoints[2];
        robotAxisPoints[2] = tempRBPoint;
        double[] currentRobotX = new double[4], currentRobotY = new double[4];
        for(int i = 0; i < 4; i++){
            RobotPoint2D transferredPoint = XYPlaneCalculations.getAbsolutePosition(currentRobotPosition,robotAxisPoints[i]);
            transferredPoint.X *= XYPlaneCalculations.INCH_PER_CM;
            transferredPoint.Y *= XYPlaneCalculations.INCH_PER_CM;
            currentRobotX[i] = transferredPoint.X;
            currentRobotY[i] = transferredPoint.Y;
        }
        RobotPoint2D robotFrontPoint = XYPlaneCalculations.getAbsolutePosition(currentRobotPosition,new RobotPoint2D(ElysiumSettings.PHYSICAL_CENTER_TO_FRONT,0));
        RobotPoint2D currentRobotPositionInch = new RobotPoint2D(currentRobotPosition.X * XYPlaneCalculations.INCH_PER_CM, currentRobotPosition.Y * XYPlaneCalculations.INCH_PER_CM);
        robotFrontPoint.X *= XYPlaneCalculations.INCH_PER_CM;
        robotFrontPoint.Y *= XYPlaneCalculations.INCH_PER_CM;
        canvas.setStroke(this.robotDrawColor);
        canvas.setFill(this.robotDrawColor);
        canvas.strokePolygon(currentRobotX,currentRobotY);
        canvas.strokeLine(currentRobotPositionInch.X,currentRobotPositionInch.Y,robotFrontPoint.X,robotFrontPoint.Y);
    }

    public RobotPose2D getCurrentPosition(){
        Pose2d currentEstimate = this.chassis.getPoseEstimate();
        return new RobotPose2D(currentEstimate.getX(), currentEstimate.getY(),Math.toDegrees(currentEstimate.getHeading()));
    }

    public void setCurrentPosition(RobotPose2D position){
        this.chassis.setPoseEstimate(new Pose2d(position.X,position.Y,Math.toRadians(position.getRotationZ())));
    }
}
