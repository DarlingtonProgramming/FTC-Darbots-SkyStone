package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.RoadRunner.drive.localizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.odometry.DistanceSensorEnhancedOdometry;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Autonomous.ElysiumAutoBase;
import org.jetbrains.annotations.NotNull;

public class RoadRunnerDistanceEnhancedLocalizer implements Localizer {
    public ElysiumAutoBase autoBase = null;
    public Localizer originalLocalizer = null;

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return originalLocalizer.getPoseEstimate();
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        originalLocalizer.setPoseEstimate(pose2d);
    }

    @Override
    public void update() {
        originalLocalizer.update();
        autoBase.calibratePositionUsingDistanceSensor();
    }
}
