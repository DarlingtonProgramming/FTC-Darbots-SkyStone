package org.darbots.darbotsftclib.utilities.chassis_tuning.MecanumChassisTuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.darbotsftclib.libcore_4_5_0Pre.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.calculations.dimentional_calculation.XYPlaneCalculations;

@TeleOp(group = "DarbotsLib-Utilities", name = "MecanumChassisFactorTuning")
public class MecanumChassisTuningTeleOp extends DarbotsBasicOpMode<MecanumChassisTuningCore> {
    private MecanumChassisTuningCore m_Core;
    private RobotPose2D relativeOffsetPose;

    public void resetRelativeOffset(){
        this.relativeOffsetPose.setValues(this.getRobotCore().getChassis().getPositionTracker().getCurrentPosition());
    }

    public RobotPose2D getRelativeOffset(){
        return XYPlaneCalculations.getRelativePosition(relativeOffsetPose,this.getRobotCore().getChassis().getPositionTracker().getCurrentPosition());
    }

    @Override
    public MecanumChassisTuningCore getRobotCore() {
        return this.m_Core;
    }

    @Override
    public void hardwareInitialize() {
        this.m_Core = new MecanumChassisTuningCore("MecanumChassisTuning.log",this.hardwareMap);
        this.relativeOffsetPose = new RobotPose2D(0,0,0);
    }

    @Override
    public void hardwareDestroy() {
        this.m_Core = null;
    }

    public boolean waitForPadX(){
        while(!(this.gamepad1.x)){
            if(this.isStopRequested()){
                return false;
            }
            sleep(50);
        }
        while(this.gamepad1.x){
            if(this.isStopRequested()){
                return false;
            }
            sleep(50);
        }
        return true;
    }

    @Override
    public void RunThisOpMode() {
        telemetry.addData("Info","This is the tuning class for mecanum drivetrain. Press X button on Gamepad 1 to continue.");
        telemetry.update();
        if(!waitForPadX()){
            return;
        }
        telemetry.addData("Info","Now we will conduct a X direction factor tuning.");
        telemetry.addData("Info","You will need to push the robot forward and measure how many CMs you've pushed it forward.");
        telemetry.addData("Info","Press X on gamepad to continue");
        telemetry.update();
        if(!waitForPadX()){
            return;
        }
        this.resetRelativeOffset();
        while((!gamepad1.x)){
            if(this.isStopRequested()){
                return;
            }
            telemetry.addData("Info", "Push the robot forward to a rounded number");
            telemetry.addData("Info", "To finish, press X on gamepad");
            this.getRobotCore().updateTelemetry();
            telemetry.update();
            sleep(50);
        }
        double endXOffset = this.getRelativeOffset().X;
        telemetry.addData("Info","End of X factor tuning");
        telemetry.addData("X Factor","" + endXOffset + " / Actual Distance Pushed");
        telemetry.addData("Info", "Press X to continue");
        telemetry.update();
        if(!waitForPadX()){
            return;
        }
        telemetry.addData("Info","Now we will conduct a Y direction factor tuning.");
        telemetry.addData("Info","You will need to push the robot to the left and measure how many CMs you've pushed it to the left.");
        telemetry.addData("Info","Press X on gamepad to continue");
        telemetry.update();
        if(!waitForPadX()){
            return;
        }
        this.resetRelativeOffset();
        while((!gamepad1.x)){
            if(this.isStopRequested()){
                return;
            }
            telemetry.addData("Info", "Push the robot to the left to a rounded number");
            telemetry.addData("Info", "To finish, press X on gamepad");
            this.getRobotCore().updateTelemetry();
            telemetry.update();
            sleep(50);
        }
        double endYOffset = this.getRelativeOffset().Y;
        telemetry.addData("Info","End of Y factor tuning");
        telemetry.addData("Y Factor","" + endYOffset + " / Actual Distance Pushed");
        telemetry.addData("Info","Press X on gamepad to continue");
        telemetry.update();
        if(!waitForPadX()){
            return;
        }
        telemetry.addData("Info","Now we will conduct a Rotational factor tuning.");
        telemetry.addData("Info","You will need to Rotate the Robot Counter-Clockwise and measure how many DEGs you've rotated it.");
        telemetry.addData("Info","Press X on gamepad to continue");
        telemetry.update();
        if(!waitForPadX()){
            return;
        }
        this.resetRelativeOffset();
        while((!gamepad1.x)){
            if(this.isStopRequested()){
                return;
            }
            telemetry.addData("Info", "Rotate the robot CW to a rounded number(DEG)[<=180]");
            telemetry.addData("Info", "To finish, press X on gamepad");
            this.getRobotCore().updateTelemetry();
            telemetry.update();
            sleep(50);
        }
        double endZRot = this.getRelativeOffset().getRotationZ();
        telemetry.addData("Info","End of Z Rot factor tuning");
        telemetry.addData("Z Rot Factor","" + endZRot + " / Actual Deg Pushed");
        telemetry.addData("Info","Congrats, Tuning is done!");
        telemetry.update();
        while(this.opModeIsActive()){
            sleep(20);
        }
    }
}
