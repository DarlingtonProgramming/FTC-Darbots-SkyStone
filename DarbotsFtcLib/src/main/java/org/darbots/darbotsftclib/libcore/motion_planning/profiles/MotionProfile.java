package org.darbots.darbotsftclib.libcore.motion_planning.profiles;

import org.darbots.darbotsftclib.libcore.calculations.algebraic_calculation.OrderedValueProvider;
import org.darbots.darbotsftclib.libcore.calculations.algebraic_calculation.OrderedValueSolver;

import java.security.Provider;
import java.util.ArrayList;

public class MotionProfile {
    private ArrayList<MotionProfileSegment> m_MotionSegments;
    public double StartVelocity;

    public MotionProfile(double startVelocity){
        this.m_MotionSegments = new ArrayList<>();
        this.StartVelocity = startVelocity;
    }

    public MotionProfile(MotionProfile oldProfile){
        this.m_MotionSegments = new ArrayList<>();
        this.m_MotionSegments.addAll(oldProfile.getMotionSegments());
        this.StartVelocity = oldProfile.StartVelocity;
    }

    public ArrayList<MotionProfileSegment> getMotionSegments() {
        return m_MotionSegments;
    }

    public double getTotalDuration(){
        double durationCounter = 0;
        for(MotionProfileSegment i : m_MotionSegments){
            durationCounter += i.getDuration();
        }
        return durationCounter;
    }

    public MotionState getMotionStateAt(double time){
        double distanceCounter = 0;
        double velocityCounter = 0;
        double durationCounter = 0;

        double timeToCountInSegment = 0;
        boolean finalSegment = false;
        for(MotionProfileSegment i : m_MotionSegments){
            if(durationCounter + i.getDuration() >= time){
                timeToCountInSegment = time - durationCounter;
                finalSegment = true;
            }else{
                timeToCountInSegment = i.getDuration();
            }
            distanceCounter += i.getDistanceTravelled(velocityCounter,timeToCountInSegment);
            velocityCounter += i.getTotalDeltaSpeed(timeToCountInSegment);
            durationCounter += i.getDuration();
            if(finalSegment){
                return new MotionState(distanceCounter,velocityCounter,i.getAccelerationAt(timeToCountInSegment),i.jerk);
            }
        }
        return null;
    }

    public double getTimeAtDistance(double Distance){
        double totalDuration = this.getTotalDuration();
        OrderedValueProvider Provider = new OrderedValueProvider() {
            @Override
            public boolean orderIncremental() {
                return true;
            }

            @Override
            public double valueAt(double independentVar) {
                MotionState stateReturned = getMotionStateAt(independentVar);
                if(stateReturned != null) {
                    return stateReturned.distance;
                }else{
                    return 0;
                }
            }
        };
        double Solution = OrderedValueSolver.solve(Provider,0.01,0,totalDuration,Distance);
        return Solution;
    }

    public double getTotalDistance(){
        double totalDuration = this.getTotalDuration();
        MotionState totalMotionState = this.getMotionStateAt(totalDuration);
        double totalDistance = totalMotionState.distance;
        return totalDistance;
    }

    public void addAtEnd(MotionProfile profile){
        this.m_MotionSegments.addAll(profile.getMotionSegments());
    }

    public MotionProfile reversed(){
        MotionState endState = this.getMotionStateAt(this.getTotalDuration());
        ArrayList<MotionProfileSegment> newSegment = new ArrayList<>();
        for(MotionProfileSegment i : this.m_MotionSegments){
            newSegment.add(i.reversed());
        }
        MotionProfile reversedProfile = new MotionProfile(endState.velocity);
        reversedProfile.getMotionSegments().addAll(newSegment);
        return reversedProfile;
    }

    public MotionProfile clipped(double startDuration, double endDuration){
        double velocityCounter = 0;
        double durationCounter = 0;

        boolean startPuttingIn = false;

        double timeToCountInSegment = 0;
        boolean finalSegment = false;
        boolean startCounting = false;
        boolean firstSegment = false;
        ArrayList<MotionProfileSegment> newProfileSegments = new ArrayList<>();
        double newProfileStartSpeed = 0;
        MotionProfileSegment ProfileSegmentGettingCounted = null;
        for(MotionProfileSegment i : m_MotionSegments){
            if(durationCounter + i.getDuration() >= startDuration){
                startCounting = true;
                firstSegment = true;
                timeToCountInSegment = i.getDuration() + durationCounter - startDuration;
            }else if(durationCounter + i.getDuration() >= endDuration){
                finalSegment = true;
                timeToCountInSegment = endDuration - durationCounter;
            }
            if(firstSegment){
                ProfileSegmentGettingCounted = i.clipped(i.getDuration() - timeToCountInSegment,i.getDuration());
                newProfileStartSpeed = velocityCounter + i.getTotalDeltaSpeed(i.getDuration() - timeToCountInSegment);
            }else if(finalSegment){
                ProfileSegmentGettingCounted = i.clipped(0,timeToCountInSegment);
            }else{
                ProfileSegmentGettingCounted = i;
            }

            if(!startCounting) {
                velocityCounter += i.getTotalDeltaSpeed();
            }
            durationCounter += i.getDuration();
            if(startCounting){
                newProfileSegments.add(ProfileSegmentGettingCounted);
            }
            if(finalSegment){
                break;
            }
            firstSegment = false;
            startCounting = false;
            finalSegment = false;
        }
        if(!startCounting){
            return null;
        }
        MotionProfile newProfile = new MotionProfile(newProfileStartSpeed);
        newProfile.getMotionSegments().addAll(newProfileSegments);
        return newProfile;
    }
}
