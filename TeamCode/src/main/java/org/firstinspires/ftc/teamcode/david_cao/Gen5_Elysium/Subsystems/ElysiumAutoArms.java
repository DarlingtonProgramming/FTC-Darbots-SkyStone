package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings.ElysiumSettings;

public class ElysiumAutoArms {
    public ElysiumAutoArm leftArm;
    public ElysiumAutoArm rightArm;
    public ElysiumAutoArms(HardwareMap map){
        this.leftArm = new ElysiumAutoArm(
                map.servo.get("leftAutoArmGrabber"),
                map.servo.get("leftAutoArmRot"),
                ElysiumSettings.AUTONOMOUS_CLAW_LEFT_ALL_CLOSED_POS,
                ElysiumSettings.AUTONOMOUS_CLAW_LEFT_WIDE_OPEN_POS,
                ElysiumSettings.AUTONOMOUS_CLAW_LEFT_GRAB_STONE_POS,
                ElysiumSettings.AUTONOMOUS_CLAW_LEFT_INSIDE_POS,
                ElysiumSettings.AUTONOMOUS_CLAW_LEFT_OUTSIDE_POS,
                ElysiumSettings.AUTONOMOUS_CLAW_LEFT_REST_POS,
                ElysiumSettings.AUTONOMOUS_CLAW_LEFT_PREPARE_DROP_POS
        );
        this.rightArm = new ElysiumAutoArm(
                map.servo.get("rightAutoArmGrabber"),
                map.servo.get("rightAutoArmRot"),
                ElysiumSettings.AUTONOMOUS_CLAW_RIGHT_ALL_CLOSED_POS,
                ElysiumSettings.AUTONOMOUS_CLAW_RIGHT_WIDE_OPEN_POS,
                ElysiumSettings.AUTONOMOUS_CLAW_RIGHT_GRAB_STONE_POS,
                ElysiumSettings.AUTONOMOUS_CLAW_RIGHT_INSIDE_POS,
                ElysiumSettings.AUTONOMOUS_CLAW_RIGHT_OUTSIDE_POS,
                ElysiumSettings.AUTONOMOUS_CLAW_RIGHT_REST_POS,
                ElysiumSettings.AUTONOMOUS_CLAW_RIGHT_PREPARE_DROP_POS
        );
    }
}
