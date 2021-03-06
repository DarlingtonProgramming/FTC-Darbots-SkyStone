package org.darbots.darbotsftclib.testcases.ServoUsingMotorTest;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.darbots.darbotsftclib.libcore.motortypes.AndyMark2964;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorController;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorWithEncoder;
import org.darbots.darbotsftclib.libcore.sensors.servos.motor_powered_servos.RobotServoUsingMotor;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ServoUsingMotorCore extends RobotCore {
    private static final MotorType SERVO_MOTOR_TYPE = new AndyMark2964();
    private static final boolean TIME_CONTROL_ENABLED = true;
    private static final double TIME_CONTROL_FACTOR = 1.5;
    private static final double INITIALIZED_POSITION = 0.0;
    private static final double MIN_POSITION = 0.0, MAX_POSITION = 1.0;

    private RobotServoUsingMotor m_ServoUsingMotor;

    public ServoUsingMotorCore(HardwareMap Hardware) {
        super("ServoUsingMotorTest.log",Hardware);
        DcMotor ServoUsingMotor_DC = Hardware.dcMotor.get("MotorServo");
        RobotMotorController ServoUsingMotorController = new RobotMotorController(
                new RobotMotorWithEncoder(ServoUsingMotor_DC,SERVO_MOTOR_TYPE),
                TIME_CONTROL_ENABLED,
                TIME_CONTROL_FACTOR
        );
        this.m_ServoUsingMotor = new RobotServoUsingMotor(
                ServoUsingMotorController,
                INITIALIZED_POSITION,
                MIN_POSITION,
                MAX_POSITION
        );
    }

    public RobotServoUsingMotor getServoUsingMotor(){
        return this.m_ServoUsingMotor;
    }

    @Override
    protected void __stop() {
        this.m_ServoUsingMotor.deleteAllTasks();
    }

    @Override
    protected void __terminate() {
        return;
    }

    @Override
    public RobotMotionSystem getChassis() {
        return null;
    }

    @Override
    public void __updateTelemetry(Telemetry telemetry, TelemetryPacket telemetryPacket) {

    }

    @Override
    public boolean isBusy() {
        return this.m_ServoUsingMotor.isBusy();
    }

    @Override
    protected void __updateStatus() {
        this.m_ServoUsingMotor.updateStatus();
    }
}
