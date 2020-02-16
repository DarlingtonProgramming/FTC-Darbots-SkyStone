package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.utilities.chassis_tuning.MecanumChassisTuning.MecanumChassisTuningCore;

@TeleOp(group = "4100", name = "Elysium-Util-ChassisTuning")
public class ElysiumChassisTuning extends DarbotsBasicOpMode<ElysiumCore> {
    private ElysiumCore m_Core;
    private RobotPose2D relativeOffsetPose;

    public void resetRelativeOffset(){
        this.relativeOffsetPose.setValues(this.getRobotCore().getChassis().getPositionTracker().getCurrentPosition());
    }

    public RobotPose2D getRelativeOffset(){
        return XYPlaneCalculations.getRelativePosition(relativeOffsetPose,this.getRobotCore().getChassis().getPositionTracker().getCurrentPosition());
    }

    @Override
    public ElysiumCore getRobotCore() {
        return this.m_Core;
    }

    @Override
    public void hardwareInitialize() {
        this.m_Core = new ElysiumCore("ElysiumChassisTuning",this.hardwareMap,false,new RobotPose2D(0,0,0),false);
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
        }
        while(this.gamepad1.x){
            if(this.isStopRequested()){
                return false;
            }
        }
        return true;
    }

    @Override
    public void RunThisOpMode() {
        {
            TelemetryPacket packet = new TelemetryPacket();
            GlobalUtil.addTelmetryLine(this.telemetry,packet,"Info", "This is the tuning class for mecanum drivetrain. Press X button on Gamepad 1 to continue.");
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.update();
        }
        if(!waitForPadX()){
            return;
        }
        {
            TelemetryPacket packet = new TelemetryPacket();
            GlobalUtil.addTelmetryLine(this.telemetry,packet,"Info", "Now we will conduct a X direction factor tuning.");
            GlobalUtil.addTelmetryLine(this.telemetry,packet,"Info", "You will need to push the robot forward and measure how many CMs you've pushed it forward.");
            GlobalUtil.addTelmetryLine(this.telemetry,packet,"Info", "Press X on gamepad to continue");
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.update();
            
        }
        if(!waitForPadX()){
            return;
        }
        this.resetRelativeOffset();
        while((!gamepad1.x)){
            if(this.isStopRequested()){
                return;
            }
            {
                TelemetryPacket packet = this.getRobotCore().updateTelemetry();
                GlobalUtil.addTelmetryLine(this.telemetry, packet, "Info", "Push the robot forward to a rounded number");
                GlobalUtil.addTelmetryLine(this.telemetry, packet, "Info", "To finish, press X on gamepad");
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                telemetry.update();
            }
            this.m_Core.updateStatus();
        }
        double endXOffset = this.getRelativeOffset().X;
        {
            TelemetryPacket packet = new TelemetryPacket();
            GlobalUtil.addTelmetryLine(this.telemetry, packet, "Info", "End of X factor tuning");
            GlobalUtil.addTelmetryLine(this.telemetry, packet, "X Factor", "" + endXOffset + " / Actual Distance Pushed");
            GlobalUtil.addTelmetryLine(this.telemetry, packet, "Info", "Press X to continue");
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.update();
        }
        sleep(100);
        if(!waitForPadX()){
            return;
        }
        {
            TelemetryPacket packet = new TelemetryPacket();
            GlobalUtil.addTelmetryLine(this.telemetry,packet,"Info","Now we will conduct a Y direction factor tuning.");
            GlobalUtil.addTelmetryLine(this.telemetry,packet,"Info","You will need to push the robot to the left and measure how many CMs you've pushed it to the left.");
            GlobalUtil.addTelmetryLine(this.telemetry,packet,"Info","Press X on gamepad to continue");
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.update();
        }

        if(!waitForPadX()){
            return;
        }
        this.resetRelativeOffset();
        while((!gamepad1.x)){
            if(this.isStopRequested()){
                return;
            }
            {
                TelemetryPacket packet = this.getRobotCore().updateTelemetry();
                GlobalUtil.addTelmetryLine(this.telemetry, packet, "Info", "Push the robot to the left to a rounded number");
                GlobalUtil.addTelmetryLine(this.telemetry, packet, "Info", "To finish, press X on gamepad");
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                telemetry.update();
            }
            this.m_Core.updateStatus();
        }
        double endYOffset = this.getRelativeOffset().Y;
        {
            TelemetryPacket packet = new TelemetryPacket();
            GlobalUtil.addTelmetryLine(this.telemetry, packet, "Info", "End of Y factor tuning");
            GlobalUtil.addTelmetryLine(this.telemetry, packet, "Y Factor", "" + endYOffset + " / Actual Distance Pushed");
            GlobalUtil.addTelmetryLine(this.telemetry, packet, "Info", "Press X on gamepad to continue");
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.update();
        }
        sleep(100);
        if(!waitForPadX()){
            return;
        }
        {
            TelemetryPacket packet = new TelemetryPacket();
            GlobalUtil.addTelmetryLine(this.telemetry, packet, "Info", "Now we will conduct a Rotational factor tuning.");
            GlobalUtil.addTelmetryLine(this.telemetry, packet, "Info", "You will need to Rotate the Robot Counter-Clockwise and measure how many DEGs you've rotated it.");
            GlobalUtil.addTelmetryLine(this.telemetry, packet, "Info", "Press X on gamepad to continue");
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.update();
        }
        if(!waitForPadX()){
            return;
        }
        this.resetRelativeOffset();
        while((!gamepad1.x)){
            if(this.isStopRequested()){
                return;
            }
            {
                TelemetryPacket packet = this.getRobotCore().updateTelemetry();
                GlobalUtil.addTelmetryLine(this.telemetry, packet, "Info", "Rotate the robot CW to a rounded number(DEG)[<=180]");
                GlobalUtil.addTelmetryLine(this.telemetry, packet, "Info", "To finish, press X on gamepad");
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                telemetry.update();
            }
            sleep(50);
        }
        double endZRot = this.getRelativeOffset().getRotationZ();
        {
            TelemetryPacket packet = new TelemetryPacket();
            GlobalUtil.addTelmetryLine(this.telemetry, packet, "Info", "End of Z Rot factor tuning");
            GlobalUtil.addTelmetryLine(this.telemetry, packet, "Z Rot Factor", "" + endZRot + " / Actual Deg Pushed");
            GlobalUtil.addTelmetryLine(this.telemetry, packet, "Info", "Congrats, Tuning is done!");
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.update();
        }
        while(this.opModeIsActive()){
            sleep(20);
        }
    }
}
