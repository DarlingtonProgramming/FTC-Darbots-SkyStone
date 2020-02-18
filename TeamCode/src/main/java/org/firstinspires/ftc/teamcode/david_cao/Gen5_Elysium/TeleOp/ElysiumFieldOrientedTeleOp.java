package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.TeleOp;

import org.darbots.darbotsftclib.game_specific.AllianceType;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.integratedfunctions.FieldOrientedMovementTeleOpControl;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings.ElysiumTeleOpSettings;

public abstract class ElysiumFieldOrientedTeleOp extends ElysiumTeleOp {
    public FieldOrientedMovementTeleOpControl fieldControl;
    public double playerRotation;

    public ElysiumFieldOrientedTeleOp(double playerRotation){
        this.playerRotation = playerRotation;
    }

    @Override
    public void hardwareInitialize(){
        super.hardwareInitialize();
        fieldControl = new FieldOrientedMovementTeleOpControl((float) playerRotation) {
            @Override
            public float getCurrentAngleCW() {
                return (float) getRobotCore().getChassis().getCurrentPosition().getRotationZ();
            }
        };
    }
    @Override
    public void driveControl(){
        double controlX = 0, controlY = 0, controlRotZ = 0;
        if(Math.abs(gamepad1.left_stick_y) >= ElysiumTeleOpSettings.GAMEPAD_THRESEHOLD) {
            controlX = -gamepad1.left_stick_y;
        }
        if(Math.abs(gamepad1.left_stick_x) >= ElysiumTeleOpSettings.GAMEPAD_THRESEHOLD) {
            controlY = -gamepad1.left_stick_x;
        }
        if(Math.abs(gamepad1.right_stick_x) >= ElysiumTeleOpSettings.GAMEPAD_THRESEHOLD) {
            controlRotZ = -gamepad1.right_stick_x;
        }

        RobotVector2D fieldOrientedControlSpeed = this.fieldControl.getRobotSpeed(new RobotVector2D(controlX,controlY,controlRotZ));

        this.teleOpTask.xSpeedNormalized = fieldOrientedControlSpeed.X * ElysiumTeleOpSettings.CHASSIS_SPEED_X_FACTOR;
        this.teleOpTask.ySpeedNormalized = fieldOrientedControlSpeed.Y * ElysiumTeleOpSettings.CHASSIS_SPEED_Y_FACTOR;
        this.teleOpTask.zRotSpeedNormalized = fieldOrientedControlSpeed.getRotationZ() * ElysiumTeleOpSettings.CHASSIS_SPEED_ROT_FACTOR;
    }
}
