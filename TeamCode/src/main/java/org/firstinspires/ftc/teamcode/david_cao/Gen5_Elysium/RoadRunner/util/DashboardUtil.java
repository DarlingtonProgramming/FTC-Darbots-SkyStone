package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.RoadRunner.util;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings.ElysiumSettings;

/**
 * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
 */
public class DashboardUtil {
    private static final double DEFAULT_RESOLUTION = 2.0; // distance units; presumed inches
    private static final double ROBOT_RADIUS = (ElysiumSettings.PHYSICAL_WIDTH / 2.0) * XYPlaneCalculations.INCH_PER_CM; // in


    public static void drawSampledPath(Canvas canvas, Path path, double resolution) {
        int samples = (int) Math.ceil(path.length() / resolution);
        double[] xPoints = new double[samples];
        double[] yPoints = new double[samples];
        double dx = path.length() / (samples - 1);
        for (int i = 0; i < samples; i++) {
            double displacement = i * dx;
            Pose2d pose = path.get(displacement);
            xPoints[i] = pose.getX() * XYPlaneCalculations.INCH_PER_CM;
            yPoints[i] = pose.getY() * XYPlaneCalculations.INCH_PER_CM;
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path) {
        drawSampledPath(canvas, path, DEFAULT_RESOLUTION);
    }

    public static void drawRobot(Canvas canvas, Pose2d pose) {
        double x = pose.getX() * XYPlaneCalculations.INCH_PER_CM;
        double y = pose.getY() * XYPlaneCalculations.INCH_PER_CM;
        canvas.strokeCircle(x, y, ROBOT_RADIUS);
        Vector2d v = pose.headingVec().times(ROBOT_RADIUS);
        double x1 = x + v.getX() / 2, y1 = y + v.getY() / 2;
        double x2 = x + v.getX(), y2 = y + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }
}