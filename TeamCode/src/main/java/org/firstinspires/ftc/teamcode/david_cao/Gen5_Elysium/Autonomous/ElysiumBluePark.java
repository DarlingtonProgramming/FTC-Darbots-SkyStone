package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Autonomous;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.odometry.DistanceSensorEnhancedOdometry;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.ElysiumAutoCore;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings.ElysiumAutonomousSettings;

public class ElysiumBluePark extends ElysiumAutoBase {
    private ElysiumAutoCore m_Core;
    @Override
    public DistanceSensorEnhancedOdometry.DistanceSensorOdometerSwitchType getDistanceSensorSwitchType(RobotPose2D currentPosition) {
        return DistanceSensorEnhancedOdometry.DistanceSensorOdometerSwitchType.NONE;
    }

    @Override
    public void __hardwareInit() {
        this.m_Core = new ElysiumAutoCore("BluePark.log",this.hardwareMap,false, ElysiumAutonomousSettings.BLUE_AUTO_PARK_POSE,false);
    }

    @Override
    public void __hardwareDestroy() {

    }

    @Override
    public void __RunOpMode() {
        
    }

    @Override
    public ElysiumAutoCore getRobotCore() {
        return this.m_Core;
    }
}
