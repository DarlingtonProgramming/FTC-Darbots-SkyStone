package org.darbots.darbotsftclib.libcore.motion_planning.profiles;

import org.darbots.darbotsftclib.libcore.calculations.algebraic_calculation.OrderedValueProvider;
import org.darbots.darbotsftclib.libcore.calculations.algebraic_calculation.OrderedValueSolver;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.MotionSystemConstraints;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotPath;

public class MotionProfileGenerator {
    public static final double PATH_DISTANCE_ERROR_MARGIN = 0.01;
    public static MotionProfile generatePathMotionProfile(MotionSystemConstraints constraints, RobotPath Path, double startVelocity, double cruiseVelocity, double endVelocity){
        return generateMotionProfile(constraints.maximumLinearSpeed,constraints.maximumLinearAcceleration,constraints.maximumLinearJerk,Path.getTotalDistance(),startVelocity,cruiseVelocity,endVelocity);
    }
    public static MotionProfile generateAngularMotionProfile(MotionSystemConstraints constraints, double degToTurn, double startVelocity, double cruiseVelocity, double endVelocity){
        if(degToTurn < 0){
            return generateMotionProfile(constraints.maximumAngularSpeed,constraints.maximumAngularAcceleration,constraints.maximumAngularJerk,-degToTurn,startVelocity,cruiseVelocity,endVelocity).negative();
        }else{
            return generateMotionProfile(constraints.maximumAngularSpeed,constraints.maximumAngularAcceleration,constraints.maximumAngularJerk,degToTurn,startVelocity,cruiseVelocity,endVelocity);
        }
    }
    public static MotionProfile generateMotionProfile(double maxVelocity, double maxAcceleration, double maxJerk, double PathTotalDistance, double startVelocity, double cruiseVelocity, double endVelocity){
        startVelocity = Math.abs(startVelocity);
        cruiseVelocity = Math.abs(cruiseVelocity);
        endVelocity = Math.abs(endVelocity);

        MotionProfile accelerateProfile = generateMotionProfileFromOneSpeedToAnother(maxVelocity,maxAcceleration,maxJerk,0,0, startVelocity,cruiseVelocity);
        MotionProfile decelerateProfile = generateMotionProfile(maxVelocity,maxAcceleration,maxJerk,0,0, cruiseVelocity,endVelocity);
        double accelerateTotalDuration = accelerateProfile.getTotalDuration();
        double decelerateTotalDuration = decelerateProfile.getTotalDuration();
        MotionState accelerateEndState = accelerateProfile.getMotionStateAt(accelerateTotalDuration);
        MotionState decelerateEndState = decelerateProfile.getMotionStateAt(decelerateTotalDuration);
        double accelerateAndDecelerateDistance = accelerateEndState.distance + decelerateEndState.distance;
        if(accelerateAndDecelerateDistance < PathTotalDistance){
            MotionProfile returnProfile = new MotionProfile(startVelocity);
            double cruiseTime = (PathTotalDistance - accelerateAndDecelerateDistance) / cruiseVelocity;
            MotionProfileSegment cruiseSegment = new MotionProfileSegment(0,0,cruiseTime);
            returnProfile.addAtEnd(accelerateProfile);
            returnProfile.addAtEnd(cruiseSegment);
            returnProfile.addAtEnd(decelerateProfile);
            return returnProfile;
        }else if(Math.abs(accelerateAndDecelerateDistance - PathTotalDistance) <= PATH_DISTANCE_ERROR_MARGIN){
            MotionProfile returnProfile = new MotionProfile(startVelocity);
            returnProfile.addAtEnd(accelerateProfile);
            returnProfile.addAtEnd(decelerateProfile);
            return returnProfile;
        }else{
            double startEndVMin = Math.min(startVelocity,endVelocity);
            double startEndVMax = Math.max(startVelocity,endVelocity);
            if(cruiseVelocity >= startEndVMin && cruiseVelocity <= startEndVMax){
                MotionProfile returnProfile = new MotionProfile(cruiseVelocity);
                double cruiseTime = PathTotalDistance / cruiseVelocity;
                MotionProfileSegment cruiseSegment = new MotionProfileSegment(0,0,cruiseTime);
                returnProfile.addAtEnd(cruiseSegment);
                return returnProfile;
            }else{
                //try to lower / rise cruise speed and see if we can achieve anything better than just cruise at cruise speed.
                final double finalMaxVelocity = maxVelocity, finalMaxAccel = maxAcceleration, finalMaxJerk = maxJerk;
                final double finalStartVelocity = startVelocity;
                final double finalEndVelocity = endVelocity;
                OrderedValueProvider valueProvider = new OrderedValueProvider() {
                    @Override
                    public boolean orderIncremental() {
                        return true;
                    }

                    @Override
                    public double valueAt(double independentVar) {
                        MotionProfile accelerateProfile = generateMotionProfileFromOneSpeedToAnother(finalMaxVelocity,finalMaxAccel,finalMaxJerk,0,0, finalStartVelocity,independentVar);
                        MotionProfile decelerateProfile = generateMotionProfileFromOneSpeedToAnother(finalMaxVelocity,finalMaxAccel,finalMaxJerk,0,0, independentVar,finalEndVelocity);
                        double accelerateTotalDuration = accelerateProfile.getTotalDuration();
                        double decelerateTotalDuration = decelerateProfile.getTotalDuration();
                        MotionState accelerateEndState = accelerateProfile.getMotionStateAt(accelerateTotalDuration);
                        MotionState decelerateEndState = decelerateProfile.getMotionStateAt(decelerateTotalDuration);
                        double accelerateAndDecelerateDistance = accelerateEndState.distance + decelerateEndState.distance;
                        return accelerateAndDecelerateDistance;
                    }
                };
                double solverMinCruise = cruiseVelocity < startEndVMin ? 0 : startEndVMax;
                double solverMaxCruise = cruiseVelocity < startEndVMin ? startEndVMin : cruiseVelocity;
                double solvedCruiseSpeed = OrderedValueSolver.solve(valueProvider,PATH_DISTANCE_ERROR_MARGIN,solverMinCruise,solverMaxCruise,PathTotalDistance);
                if(solvedCruiseSpeed == OrderedValueSolver.RESULT_NOSOLUTION){
                    MotionProfile returnProfile = new MotionProfile(cruiseVelocity);
                    double cruiseTime = PathTotalDistance / cruiseVelocity;
                    MotionProfileSegment cruiseSegment = new MotionProfileSegment(0,0,cruiseTime);
                    returnProfile.addAtEnd(cruiseSegment);
                    return returnProfile;
                }else{
                    MotionProfile newAccelerateProfile = generateMotionProfileFromOneSpeedToAnother(maxVelocity,maxAcceleration,maxJerk,0,0, startVelocity,solvedCruiseSpeed);
                    MotionProfile newDecelerateProfile = generateMotionProfileFromOneSpeedToAnother(maxVelocity,maxAcceleration,maxJerk,0,0, solvedCruiseSpeed,endVelocity);
                    MotionProfile returnProfile = new MotionProfile(startVelocity);
                    returnProfile.addAtEnd(newAccelerateProfile);
                    returnProfile.addAtEnd(newDecelerateProfile);
                    return returnProfile;
                }
            }
        }
    }
    public static MotionProfile generateMotionProfileWithConstantVelocity(double constantVelocity, double duration){
        MotionProfile returnProfile = new MotionProfile(constantVelocity);
        MotionProfileSegment segment = new MotionProfileSegment(0,0,duration);
        returnProfile.addAtEnd(segment);
        return returnProfile;
    }
    public static MotionProfile generateMotionProfileFromOneSpeedToAnother(double maxVelocity, double maxAccel, double maxJerk, double startAcceleration, double endAcceleration, double startVelocity, double endVelocity){
        if(endVelocity < startVelocity){
            return generateMotionProfileFromOneSpeedToAnother(maxVelocity,maxAccel,maxJerk,startAcceleration,endAcceleration,endVelocity,startVelocity).reversed();
        }
        return __generateMotionProfileFromOneSpeedToAnother_JERKUNLIMITED(maxVelocity,maxAccel,maxJerk,startVelocity,endVelocity);
    }
    public static MotionProfile __generateMotionProfileFromOneSpeedToAnother_JERKUNLIMITED(double maxVelocity, double maxAccel, double maxJerk, double startVelocity, double endVelocity){
        if(endVelocity < startVelocity){
            return __generateMotionProfileFromOneSpeedToAnother_JERKUNLIMITED(maxVelocity,maxAccel,maxJerk,endVelocity,startVelocity).reversed();
        }

        double currentVelocity = startVelocity;

        double Tvelocity = 0;
        Tvelocity = (endVelocity - currentVelocity) /  maxAccel;
        MotionProfileSegment segmentVelocityToEndSpeed = null;
        if(Tvelocity != 0){
            segmentVelocityToEndSpeed = new MotionProfileSegment(maxAccel,0,Tvelocity);
        }
        MotionProfile returnProfile = new MotionProfile(startVelocity);
        if(segmentVelocityToEndSpeed != null){
            returnProfile.addAtEnd(segmentVelocityToEndSpeed);
        }
        return returnProfile;
    }

}
