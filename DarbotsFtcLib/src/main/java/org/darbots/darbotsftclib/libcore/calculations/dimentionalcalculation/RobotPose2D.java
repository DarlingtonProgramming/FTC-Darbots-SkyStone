/*
MIT License

Copyright (c) 2018 DarBots Collaborators

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/
package org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation;


public class RobotPose2D {
    protected double m_X;
    protected double m_Y;
    protected double m_RotationZ;
    public RobotPose2D(double X, double Y, double YRotation){
        this.m_X = X;
        this.m_Y = Y;
        this.m_RotationZ = XYPlaneCalculations.normalizeDeg(YRotation);
    }
    public RobotPose2D(RobotPoint2D Point, double ZRotation) {
        this.m_X = Point.X;
        this.m_Y = Point.Y;
        this.m_RotationZ = ZRotation;
    }
    public RobotPose2D(RobotPose2D Pos2D){
        this.m_X = Pos2D.m_X;
        this.m_Y = Pos2D.m_Y;
        this.m_RotationZ = Pos2D.m_RotationZ;
    }
    public double getX(){
        return this.m_X;
    }
    public void setX(double X){
        this.m_X = X;
    }
    public double getY(){
        return this.m_Y;
    }
    public void setY(double Y){
        this.m_Y = Y;
    }
    public double getDistanceToOrigin(){
        return (Math.sqrt(Math.pow(this.getX(),2) + Math.pow(this.getY(),2)));
    }
    public double getRotationZ(){
        return this.m_RotationZ;
    }
    public void setRotationZ(double RotationZ){
        this.m_RotationZ = XYPlaneCalculations.normalizeDeg(RotationZ);
    }
    public RobotPoint2D toPoint2D() {
        return new RobotPoint2D(this);
    }
}
