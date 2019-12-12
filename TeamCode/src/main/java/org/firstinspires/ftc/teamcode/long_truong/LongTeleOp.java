/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.long_truong;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Long TeleOp", group="Linear Opmode")
//@Disabled
public class LongTeleOp extends LinearOpMode {
    HardwareLongBot robot = new HardwareLongBot();

//    public void stop_andResetEncoders(){
//        robot.flDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.frDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.blDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.brDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }


    public void pulling(boolean arrow1, boolean arrow2){
        if (arrow1){
            robot.pullMotor.setPower(robot.power*2);
        }

        else if (arrow2){
            robot.pullMotor.setPower(-robot.power*2);
        }

        else {
            robot.pullMotor.setPower(0);
        }
    }

    public void grabbing(float trigger, boolean bumper){
        if (trigger > 0.25){
            robot.grabservo.setPosition(trigger);
        }
        else if (bumper){
            robot.grabservo.setPosition(0);
        }
        else {
            robot.grabservo.setPosition(0);
        }
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);

        waitForStart();
        robot.runtime.reset();

        while (opModeIsActive()) {
            //Chassis Movement Control
            double strafeSpeed = gamepad1.left_stick_x;
            double forwardSpeed = -gamepad1.left_stick_y;
            double rotSpeed = -gamepad1.right_stick_x;
            double FLMotorSpeed = -strafeSpeed - forwardSpeed + rotSpeed;
            double FRMotorSpeed = - strafeSpeed + forwardSpeed + rotSpeed;
            double BLMotorSpeed = + strafeSpeed - forwardSpeed + rotSpeed;
            double BRMotorSpeed = + strafeSpeed + forwardSpeed + rotSpeed;
            FLMotorSpeed = Range.clip(FLMotorSpeed, -1.0, 1.0);
            FRMotorSpeed = Range.clip(FRMotorSpeed, -1.0, 1.0);
            BLMotorSpeed = Range.clip(BLMotorSpeed, -1.0, 1.0);
            BRMotorSpeed = Range.clip(BRMotorSpeed, -1.0, 1.0);
            robot.flDrive.setPower(FLMotorSpeed);
            robot.frDrive.setPower(FRMotorSpeed);
            robot.blDrive.setPower(BLMotorSpeed);
            robot.brDrive.setPower(BRMotorSpeed);


            pulling(gamepad1.dpad_up, gamepad1.dpad_down);
            grabbing(gamepad1.right_trigger, gamepad1.right_bumper);


            telemetry.addData("Status", "Run Time: " + robot.runtime.toString());
            telemetry.update();

            idle();
        }
    }
}
