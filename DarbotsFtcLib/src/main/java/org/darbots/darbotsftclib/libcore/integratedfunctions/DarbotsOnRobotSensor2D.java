package org.darbots.darbotsftclib.libcore.integratedfunctions;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;

public class DarbotsOnRobotSensor2D<E> {
    public RobotPose2D OnRobotPosition;
    public E Sensor;
    public DarbotsOnRobotSensor2D(RobotPose2D onRobotPosition, E sensor){
        this.OnRobotPosition = onRobotPosition;
        this.Sensor = sensor;
    }
    public DarbotsOnRobotSensor2D(DarbotsOnRobotSensor2D<E> onRobotSensor2D){
        this.OnRobotPosition = onRobotSensor2D.OnRobotPosition;
        this.Sensor = onRobotSensor2D.Sensor;
    }
}
