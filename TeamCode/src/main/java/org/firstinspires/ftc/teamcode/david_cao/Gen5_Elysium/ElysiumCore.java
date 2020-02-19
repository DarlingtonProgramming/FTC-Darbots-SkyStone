package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.integratedfunctions.DarbotsOnRobotSensor2D;
import org.darbots.darbotsftclib.libcore.integratedfunctions.FTCMemory;
import org.darbots.darbotsftclib.libcore.motionsystems.MecanumDrivetrain;
import org.darbots.darbotsftclib.libcore.odometry.DistanceSensorEnhancedOdometry;
import org.darbots.darbotsftclib.libcore.odometry.MecanumOdometry;
import org.darbots.darbotsftclib.libcore.runtime.MovementUtil;
import org.darbots.darbotsftclib.libcore.sensors.distance_sensors.DarbotsRevDistanceSensor;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotMotion;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotWheel;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorWithEncoder;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.MotionSystemConstraints;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;
import org.darbots.darbotsftclib.libcore.templates.odometry.OdometryMethod;
import org.darbots.darbotsftclib.libcore.templates.odometry.RobotAsyncPositionTracker;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings.ElysiumSettings;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Subsystems.ElysiumAutoArms;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Subsystems.ElysiumCapstoneDelivery;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Subsystems.ElysiumIntake;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Subsystems.ElysiumOuttake;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Subsystems.ElysiumStacker;

public class ElysiumCore extends RobotCore {
    protected MecanumDrivetrain m_Chassis;
    public ElysiumCapstoneDelivery capstoneDeliverySubSystem;
    public ElysiumIntake intakeSubSystem;
    public ElysiumOuttake outtakeSubSystem;
    public ElysiumStacker stackerSubSystem;
    public ElysiumAutoArms autoArmsSubSystem;
    public MotionSystemConstraints motionSystemConstraints;

    public ElysiumCore(String logFileName, HardwareMap hardwareMap, boolean read, boolean initChassis, RobotPose2D initialPose, boolean distanceEnhancedLocalization) {
        super(logFileName, hardwareMap);
        if(initChassis) {
            this.initializeDriveTrain(hardwareMap);
            this.getChassis().setDrawRobotLength(ElysiumSettings.PHYSICAL_LENGTH);
            this.getChassis().setDrawRobotWidth(ElysiumSettings.PHYSICAL_WIDTH);

            OdometryMethod odometryMethod = null;
            if (distanceEnhancedLocalization) {
                odometryMethod = this.initializeDistanceSensorEnhancedTracker(hardwareMap);
            } else {
                odometryMethod = this.initializeNoDistanceSensorTracker();
            }
            RobotAsyncPositionTracker posTracker = new RobotAsyncPositionTracker(odometryMethod, initialPose);
            if (read) {
                this.read(initialPose);
            }
            posTracker.setGyroProvider(this.getGyro());
            posTracker.setDistanceFactors(ElysiumSettings.CHASSIS_FACTORS);
            this.getChassis().setPositionTracker(posTracker);
            posTracker.start();
            this.motionSystemConstraints = this.getChassis().getMotionSystemConstraints(ElysiumSettings.CHASSIS_MAXIMUM_ACCEL, 0, ElysiumSettings.CHASSIS_MAXIMUM_ANGULAR_ACCEL_DEG, 0);
            MovementUtil.drivetrain_constraints = this.motionSystemConstraints;
        }

        this.capstoneDeliverySubSystem = new ElysiumCapstoneDelivery(hardwareMap);
        this.intakeSubSystem = new ElysiumIntake(hardwareMap);
        this.outtakeSubSystem = new ElysiumOuttake(hardwareMap);
        this.stackerSubSystem = new ElysiumStacker(hardwareMap);
        this.autoArmsSubSystem = new ElysiumAutoArms(hardwareMap);
    }

    private void initializeDriveTrain(HardwareMap map){
        double HALF_LENGTH = ElysiumSettings.CHASSIS_LENGTH / 2.0;
        double HALF_WIDTH = ElysiumSettings.CHASSIS_WIDTH / 2.0;
        RobotMotorWithEncoder LTMotor = new RobotMotorWithEncoder(map.dcMotor.get("LF"),ElysiumSettings.CHASSIS_MOTOR_TYPE);
        RobotMotorWithEncoder RTMotor = new RobotMotorWithEncoder(map.dcMotor.get("RF"),ElysiumSettings.CHASSIS_MOTOR_TYPE);
        RobotMotorWithEncoder LBMotor = new RobotMotorWithEncoder(map.dcMotor.get("LB"),ElysiumSettings.CHASSIS_MOTOR_TYPE);
        RobotMotorWithEncoder RBMotor = new RobotMotorWithEncoder(map.dcMotor.get("RB"),ElysiumSettings.CHASSIS_MOTOR_TYPE);
        RobotWheel LTWheel = new RobotWheel(new RobotPose2D(HALF_LENGTH,HALF_WIDTH,135),ElysiumSettings.CHASSIS_WHEEL_RADIUS);
        RobotWheel RTWheel = new RobotWheel(new RobotPose2D(HALF_LENGTH,-HALF_WIDTH,45),ElysiumSettings.CHASSIS_WHEEL_RADIUS);
        RobotWheel LBWheel = new RobotWheel(new RobotPose2D(-HALF_LENGTH,HALF_WIDTH,-135),ElysiumSettings.CHASSIS_WHEEL_RADIUS);
        RobotWheel RBWheel = new RobotWheel(new RobotPose2D(-HALF_LENGTH,-HALF_WIDTH,-45),ElysiumSettings.CHASSIS_WHEEL_RADIUS);
        RobotMotion LT = new RobotMotion(LTMotor,LTWheel);
        RobotMotion RT = new RobotMotion(RTMotor,RTWheel);
        RobotMotion LB = new RobotMotion(LBMotor,LBWheel);
        RobotMotion RB = new RobotMotion(RBMotor,RBWheel);
        this.m_Chassis = new MecanumDrivetrain(null,LT,RT,LB,RB);
        this.m_Chassis.setLinearXMotionDistanceFactor(ElysiumSettings.CHASSIS_FACTORS.X);
        this.m_Chassis.setLinearYMotionDistanceFactor(ElysiumSettings.CHASSIS_FACTORS.Y);
        this.m_Chassis.setRotationalMotionDistanceFactor(ElysiumSettings.CHASSIS_FACTORS.getRotationZ());
    }

    private OdometryMethod initializeNoDistanceSensorTracker(){
        return new MecanumOdometry(this.m_Chassis);
    }

    private OdometryMethod initializeDistanceSensorEnhancedTracker(HardwareMap map){
        DarbotsOnRobotSensor2D<DarbotsRevDistanceSensor> FrontSensor, LeftSensor, BackSensor, RightSensor;
        FrontSensor = new DarbotsOnRobotSensor2D<DarbotsRevDistanceSensor>(ElysiumSettings.LOCALIZATION_FRONTDISTSENSOR_POS,new DarbotsRevDistanceSensor(map.get(DistanceSensor.class,"frontDistanceSensor")));
        LeftSensor = new DarbotsOnRobotSensor2D<DarbotsRevDistanceSensor>(ElysiumSettings.LOCALIZATION_LEFTDISTSENSOR_POS,new DarbotsRevDistanceSensor(map.get(DistanceSensor.class,"leftDistanceSensor")));
        BackSensor = new DarbotsOnRobotSensor2D<DarbotsRevDistanceSensor>(ElysiumSettings.LOCALIZATION_BACKDISTSENSOR_POS,new DarbotsRevDistanceSensor(map.get(DistanceSensor.class,"backDistanceSensor")));
        RightSensor = new DarbotsOnRobotSensor2D<DarbotsRevDistanceSensor>(ElysiumSettings.LOCALIZATION_RIGHTDISTSENSOR_POS,new DarbotsRevDistanceSensor(map.get(DistanceSensor.class,"rightDistanceSensor")));
        OdometryMethod originalOdometry = initializeNoDistanceSensorTracker();
        OdometryMethod distanceEnhancedOdometry = new DistanceSensorEnhancedOdometry(originalOdometry,FrontSensor,LeftSensor,BackSensor,RightSensor);
        return distanceEnhancedOdometry;
    }

    public void save(){
        this.capstoneDeliverySubSystem.save();
        this.outtakeSubSystem.save();
        this.stackerSubSystem.save();
        if(this.getChassis() != null) {
            RobotPose2D currentPosition = this.getChassis().getCurrentPosition();
            FTCMemory.setSetting("ElysiumRobotPose", currentPosition);
        }
        FTCMemory.saveMemoryMapToFile();
    }

    public void read(RobotPose2D defaultPose){
        this.capstoneDeliverySubSystem.read();
        this.outtakeSubSystem.read();
        this.stackerSubSystem.read();
        if(this.getChassis() != null) {
            try {
                RobotPose2D readPosition = FTCMemory.getSetting("ElysiumRobotPose", defaultPose);
                this.getChassis().getPositionTracker().setCurrentPosition(readPosition);
            } catch (Exception e) {
                //do nothing
            }
        }
    }

    @Override
    protected void __stop() {
        if(this.getChassis() != null) {
            this.getChassis().stop();
        }
        this.stackerSubSystem.stop();
        this.intakeSubSystem.stop();
        this.outtakeSubSystem.stop();
        this.capstoneDeliverySubSystem.stop();
    }

    @Override
    protected void __terminate() {
        if(this.getChassis() != null) {
            this.getChassis().terminate();
        }
    }

    @Override
    public RobotMotionSystem getChassis() {
        return this.m_Chassis;
    }

    @Override
    protected void __updateStatus() {
        if(this.getChassis() != null) {
            this.getChassis().updateStatus();
        }
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

    @Override
    public boolean isBusy() {
        return this.capstoneDeliverySubSystem.isBusy() || this.intakeSubSystem.isBusy() || this.outtakeSubSystem.isBusy() || this.stackerSubSystem.isBusy();
    }
}
