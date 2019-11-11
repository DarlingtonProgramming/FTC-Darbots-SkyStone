package org.darbots.darbotsftclib.libcore.templates.chassis_related;

public abstract class RobotMotionSystemFixedYDistanceTask extends RobotMotionSystemTask {
    private double m_ZDistance;
    private double m_Speed;

    public RobotMotionSystemFixedYDistanceTask(double ZDistance, double Speed){
        this.m_ZDistance = ZDistance;
        this.m_Speed = Speed;
    }
    public RobotMotionSystemFixedYDistanceTask(RobotMotionSystemFixedYDistanceTask Task){
        super(Task);
        this.m_ZDistance = Task.m_ZDistance;
        this.m_Speed = Task.m_Speed;
    }
    public double getSpeed(){
        return this.m_Speed;
    }
    public void setSpeed(double Speed){
        this.m_Speed = Math.abs(Speed);
    }
    public double getZDistance(){
        return this.m_ZDistance;
    }
    public void setZDistance(double Distance){
        this.m_ZDistance = Distance;
    }
    @Override
    public String getTaskDetailString(){
        String result = "TaskType: MotionSystemFixedZDistanceTask, ";
        result += "Speed: " + this.getSpeed() + ", ";
        result += "ZDistance: " + this.getZDistance();
        return result;
    }
}
