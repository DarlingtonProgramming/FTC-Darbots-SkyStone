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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class RobotWebcamCamera implements RobotCamera {
    private VuforiaLocalizer m_Vuforia;
    private String m_VuforiaKey;
    private OpMode m_ControllerOp;
    private String m_WebcamConfigurationName;
    private WebcamName m_WebcamName;
    private boolean m_Preview;
    public RobotWebcamCamera(@NonNull OpMode controllerOp, boolean preview, String WebCamConfigName, String VuforiaKey){
        this.m_ControllerOp = controllerOp;
        this.m_WebcamConfigurationName = WebCamConfigName;
        this.m_VuforiaKey = VuforiaKey;
        this.m_Preview = preview;
        this.createVuforia();
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
            int cameraMonitorViewId = m_ControllerOp.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", m_ControllerOp.hardwareMap.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        }else {
            parameters = new VuforiaLocalizer.Parameters();
        }

        parameters.vuforiaLicenseKey = m_VuforiaKey;
        m_WebcamName = this.m_ControllerOp.hardwareMap.get(WebcamName.class, this.m_WebcamConfigurationName);
        parameters.cameraName = m_WebcamName;

        //  Instantiate the Vuforia engine
        this.m_Vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    public WebcamName getWebcam(){
        return m_WebcamName;
    }
}
