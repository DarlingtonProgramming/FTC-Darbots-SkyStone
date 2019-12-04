package org.darbots.darbotsftclib.libcore.motion_planning.profiles;

import org.darbots.darbotsftclib.libcore.calculations.algebraic_calculation.OrderedValueProvider;
import org.darbots.darbotsftclib.libcore.calculations.algebraic_calculation.OrderedValueSolver;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.MotionSystemConstraints;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotPath;

public class MotionProfileGenerator {
    public static final double PATH_DISTANCE_ERROR_MARGIN = 0.01;
    public static MotionProfile generateMotionProfile(MotionSystemConstraints constraints, RobotPath Path, double startVelocity, double cruiseVelocity, double endVelocity){
        MotionProfile accelerateProfile = generateMotionProfile(constraints,0,0, startVelocity,cruiseVelocity);
        MotionProfile decelerateProfile = generateMotionProfile(constraints,0,0, cruiseVelocity,endVelocity);
        double accelerateTotalDuration = accelerateProfile.getTotalDuration();
        double decelerateTotalDuration = decelerateProfile.getTotalDuration();
        MotionState accelerateEndState = accelerateProfile.getMotionStateAt(accelerateTotalDuration);
        MotionState decelerateEndState = decelerateProfile.getMotionStateAt(decelerateTotalDuration);
        double accelerateAnddecelerateDistance = accelerateEndState.distance + decelerateEndState.distance;
        double PathTotalDistance = Path.getTotalDistance();
        if(accelerateAnddecelerateDistance < PathTotalDistance){
            MotionProfile returnProfile = new MotionProfile(startVelocity);
            double cruiseTime = (PathTotalDistance - accelerateAnddecelerateDistance) / cruiseVelocity;
            MotionProfileSegment cruiseSegment = new MotionProfileSegment(0,0,cruiseTime);
            returnProfile.addAtEnd(accelerateProfile);
            returnProfile.addAtEnd(cruiseSegment);
            returnProfile.addAtEnd(decelerateProfile);
            return returnProfile;
        }else if(Math.abs(accelerateAnddecelerateDistance - PathTotalDistance) <= PATH_DISTANCE_ERROR_MARGIN){
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
                //try to lower cruise speed and see if we can achieve anything better than just cruise at cruise speed.
                final MotionSystemConstraints finalConstraints = constraints;
                final double finalStartVelocity = startVelocity;
                final double finalEndVelocity = endVelocity;
                OrderedValueProvider valueProvider = new OrderedValueProvider() {
                    @Override
                    public boolean orderIncremental() {
                        return true;
                    }

                    @Override
                    public double valueAt(double independentVar) {
                        MotionProfile accelerateProfile = generateMotionProfile(finalConstraints,0,0, finalStartVelocity,independentVar);
                        MotionProfile decelerateProfile = generateMotionProfile(finalConstraints,0,0, independentVar,finalEndVelocity);
                        double accelerateTotalDuration = accelerateProfile.getTotalDuration();
                        double decelerateTotalDuration = decelerateProfile.getTotalDuration();
                        MotionState accelerateEndState = accelerateProfile.getMotionStateAt(accelerateTotalDuration);
                        MotionState decelerateEndState = decelerateProfile.getMotionStateAt(decelerateTotalDuration);
                        double accelerateAnddecelerateDistance = accelerateEndState.distance + decelerateEndState.distance;
                        return accelerateAnddecelerateDistance;
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
                    MotionProfile newAccelerateProfile = generateMotionProfile(constraints,0,0, startVelocity,solvedCruiseSpeed);
                    MotionProfile newDecelerateProfile = generateMotionProfile(constraints,0,0, solvedCruiseSpeed,endVelocity);
                    MotionProfile returnProfile = new MotionProfile(startVelocity);
                    returnProfile.addAtEnd(newAccelerateProfile);
                    returnProfile.addAtEnd(newDecelerateProfile);
                    return returnProfile;
                }
            }
        }
    }
    public static MotionProfile generateMotionProfile(double constantVelocity, double duration){
        MotionProfile returnProfile = new MotionProfile(constantVelocity);
        MotionProfileSegment segment = new MotionProfileSegment(0,0,duration);
        returnProfile.addAtEnd(segment);
        return returnProfile;
    }
    public static MotionProfile generateMotionProfile(MotionSystemConstraints constraints, double startAcceleration, double endAcceleration, double startVelocity, double endVelocity){
        if(endVelocity < startVelocity){
            return generateMotionProfile(constraints,startAcceleration,endAcceleration,endVelocity,startVelocity).reversed();
        }
        return __generateMotionProfile_JERKUNLIMITED(constraints,startVelocity,endVelocity);
    }
    public static MotionProfile __generateMotionProfile_JERKUNLIMITED(MotionSystemConstraints constraints, double startVelocity, double endVelocity){
        if(endVelocity < startVelocity){
            return __generateMotionProfile_JERKUNLIMITED(constraints,endVelocity,startVelocity).reversed();
        }
        double Taccel = 0;
        double currentVelocity = startVelocity;

        double Tvelocity = 0;
        Tvelocity = (endVelocity - currentVelocity) / constraints.maximumLinearAcceleration;
        MotionProfileSegment segmentVelocityToEndSpeed = null;
        if(Tvelocity != 0){
            segmentVelocityToEndSpeed = new MotionProfileSegment(constraints.maximumLinearAcceleration,0,Tvelocity);
        }
        MotionProfile returnProfile = new MotionProfile(startVelocity);
        if(segmentVelocityToEndSpeed != null){
            returnProfile.addAtEnd(segmentVelocityToEndSpeed);
        }
        return returnProfile;
    }
}
