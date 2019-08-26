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

package org.darbots.darbotsftclib.libcore.sensors.cameras;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.darbots.darbotsftclib.libcore.templates.RobotCamera;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class RobotOnPhoneCamera implements RobotCamera {
    private VuforiaLocalizer.CameraDirection m_CameraDirection;
    private VuforiaLocalizer m_Vuforia;
    private String m_VuforiaKey;
    private boolean m_Preview;
    private OpMode m_ControllingOpMode;
    public RobotOnPhoneCamera(@NonNull OpMode controllerOpMode, boolean preview, VuforiaLocalizer.CameraDirection CameraDirection, String VuforiaKey){
        this.m_CameraDirection = CameraDirection;
        this.m_ControllingOpMode = controllerOpMode;
        this.m_VuforiaKey = VuforiaKey;
        this.m_Preview = preview;
        this.createVuforia();
    }
    public VuforiaLocalizer.CameraDirection getCameraDirection(){
        return this.m_CameraDirection;
    }
    @Override
    public VuforiaLocalizer getVuforia() {
        return this.m_Vuforia;
    }

    @Override
    public boolean isPreview() {
        return this.m_Preview;
    }

    protected void createVuforia(){

        VuforiaLocalizer.Parameters parameters = null;
        if(this.m_Preview){
            int cameraMonitorViewId = m_ControllingOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", m_ControllingOpMode.hardwareMap.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        }else {
            parameters = new VuforiaLocalizer.Parameters();
        }

        parameters.vuforiaLicenseKey = this.m_VuforiaKey;
        parameters.cameraDirection = this.m_CameraDirection;

        //  Instantiate the Vuforia engine
        this.m_Vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
}
