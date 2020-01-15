package org.darbots.darbotsftclib.libcore.runtime;

import org.darbots.darbotsftclib.libcore.integratedfunctions.FTCFileIO;
import org.darbots.darbotsftclib.libcore.motion_planning.followers.TrajectoryFollower;
import org.darbots.darbotsftclib.libcore.motion_planning.paths.LinePath;
import org.darbots.darbotsftclib.libcore.motion_planning.trajectories.SimpleTrajectory;
import org.darbots.darbotsftclib.libcore.motion_planning.trajectories.SimpleTrajectoryGenerator;
import org.darbots.darbotsftclib.libcore.motion_planning.trajectories.TrajectoryIO;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.MotionSystemConstraints;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotPath;

import java.io.File;

public class MovementUtil {
    public static MotionSystemConstraints drivetrain_constraints = null;
    public static double resolution = 0.02;
    public static TrajectoryFollower getGoToPointTask(double X, double Y, double startSpeed, double cruiseSpeed, double endSpeed, double preferredAng){
        if(X == 0 && Y == 0){
            return null;
        }
        if(drivetrain_constraints == null){
            return null;
        }

        checkFolder();

        boolean isSavedFile = false;
        String savedFileName = null;
        File savedFile = null;
        if(GlobalRegister.runningOpMode == null || GlobalRegister.runningOpMode.getRobotCore() == null){
            isSavedFile = false;
            savedFileName = null;
            savedFile = null;
        }else{
            isSavedFile = true;
            savedFileName = GlobalRegister.runningOpMode.getRobotCore().getClass().getSimpleName() + ".linear." + X + "." + Y + "." + startSpeed + "." + cruiseSpeed + "." + endSpeed + ".trajectoryFile";
            savedFile = getTrajectoryFile(savedFileName);
            if(!savedFile.exists()){
                isSavedFile = false;
            }
        }
        SimpleTrajectory linearTrajectory = null;
        if(isSavedFile){
            linearTrajectory = TrajectoryIO.loadTrajectory(savedFile);
        }
        if(linearTrajectory == null) {
            RobotPath linearPath = new LinePath(X, Y);
            linearTrajectory = SimpleTrajectoryGenerator.generateTrajectory(
                    resolution,
                    drivetrain_constraints,
                    linearPath,
                    startSpeed,
                    cruiseSpeed,
                    endSpeed,
                    preferredAng
            );
            if(savedFile != null){
                TrajectoryIO.saveTrajectory(linearTrajectory,savedFile);
            }
        }
        return new TrajectoryFollower(linearTrajectory);
    }
    protected static void checkFolder(){
        File firstFolder = FTCFileIO.getFirstFolder();
        File trajectoryFolder = new File(firstFolder,"DarbotsTrajectories");
        if(!trajectoryFolder.isDirectory()){
            trajectoryFolder.mkdirs();
        }
    }
    protected static File getTrajectoryFile(String filename){
        File firstFolder = FTCFileIO.getFirstFolder();
        File TrajectoryDir = new File(firstFolder,"DarbotsTrajectories");
        File TrajectoryFile = new File(TrajectoryDir,filename);
        return TrajectoryFile;
    }
}
