package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.RoadRunner.drive.localizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.darbots.darbotsftclib.libcore.templates.odometry.CustomizableOdometry;
import org.darbots.darbotsftclib.libcore.templates.odometry.Robot2DPositionTracker;
import org.darbots.darbotsftclib.libcore.templates.odometry.RobotAsyncPositionTracker;
import org.darbots.darbotsftclib.libcore.templates.odometry.RobotSeparateThreadPositionTracker;
import org.jetbrains.annotations.NotNull;

public class RoadRunnerLocalizer implements Localizer {
    public CustomizableOdometry positionTracker = null;
    public RoadRunnerLocalizer(CustomizableOdometry tracker){
        positionTracker = tracker;
    }
    public RoadRunnerLocalizer(RoadRunnerLocalizer localizer){
        this.positionTracker = localizer.positionTracker;
    }
    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        this.positionTracker.updateGyroProvider();
        return new Pose2d(positionTracker.getCurrentPosition().X,positionTracker.getCurrentPosition().Y, Math.toRadians(positionTracker.getCurrentPosition().getRotationZ()));
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        positionTracker.setCurrentPosition(new RobotPose2D(pose2d.getX(),pose2d.getY(), Math.toDegrees(pose2d.getHeading())));
    }

    @Override
    public void update() {
        if(positionTracker instanceof RobotNonBlockingDevice){
            ((RobotNonBlockingDevice) positionTracker).updateStatus();
        }
    }
}
