package org.firstinspires.ftc.teamcode.david_cao.Gen4_SwanSilver_Code;

import org.darbots.darbotsftclib.libcore_4_5_0Pre.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.calculations.dimentional_calculation.XYPlaneCalculations;

public abstract class SwanSilverAutoBase extends DarbotsBasicOpMode<SwanSilverCore> {
    private SwanSilverCore m_Core;
    public boolean pointMirrored = false;

    @Override
    public SwanSilverCore getRobotCore() {
        return this.m_Core;
    }

    @Override
    public void hardwareInitialize() {
        this.m_Core = new SwanSilverCore(this.hardwareMap,"SwanSilverAutonomous.log");
    }

    @Override
    public void hardwareDestroy() {

    }

    public RobotPoint2D getWorldPoint(RobotPoint2D rawPoint){
        if(!pointMirrored){
            return rawPoint;
        }else{
            RobotPoint2D newPoint = new RobotPoint2D(rawPoint.X, -rawPoint.Y);
            return newPoint;
        }
    }

    public RobotPose2D getWorldPose(RobotPose2D rawPose){
        if(!pointMirrored){
            return rawPose;
        }else{
            RobotPose2D newPose = new RobotPose2D(rawPose.X, -rawPose.Y, -rawPose.getRotationZ());
            return newPose;
        }
    }

    public double getWorldRotation(double rawRotation){
        if(!pointMirrored){
            return rawRotation;
        }else{
            return XYPlaneCalculations.normalizeDeg(-rawRotation);
        }
    }

    public RobotPose2D getRobotPose(RobotPose2D rawWorldPose){
        return this.getRobotCore().getChassis().getPositionTracker().robotAxisFromFieldAxis(this.getWorldPose(rawWorldPose));
    }
}
