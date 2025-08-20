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

package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


@TeleOp(name="Turret", group="Linear OpMode")



public class Turret extends LinearOpMode {
//    private AnalogInput ranger;
    public DcMotorEx  leftFront;
    public DcMotorEx  rightFront;
    public DcMotorEx  leftBack;
    public DcMotorEx  rightBack;
    public IMU imu;

    public AnalogInput rangerLeft;
    public AnalogInput rangerRight;

    @Override
    public void runOpMode() {
        waitForStart();
        rangerLeft = hardwareMap.get(AnalogInput.class, "rangerLeft");
        rangerRight = hardwareMap.get(AnalogInput.class, "rangerRight");

        leftFront  = hardwareMap.get(DcMotorEx.class, "left_front");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        double facing = 0;
        double y = 0;
        double x = 0;
        double rx = 0;



        while (opModeIsActive()) {

            double shift = 0.5;
            facing = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//            y = -gamepad1.left_stick_y;
//            //double y = 0;
//            x = gamepad1.left_stick_x;
//            rx = gamepad1.right_stick_x;

            if (gamepad1.a) {
                x = 0.3;
                y = -0.5;
            }

            if (facing < 0.9) {
                rx = 0.7;
            } else if (facing > 1.1) {
                rx = -0.7;
            } else {
                rx = 0;
            }

            double rotX = x * Math.cos(-facing) - y * Math.sin(-facing);
            rotX = rotX * 1.1;
            double rotY = x * Math.sin(-facing) + y * Math.cos(-facing);

            double d = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(rx), 1);

            double lf = (rotY + rotX + rx) / d;
            double lb = (rotY - rotX + rx) / d;
            double rf = (rotY - rotX - rx) / d;
            double rb = (rotY + rotX - rx) / d;

            leftBack.setVelocity(3000 * lb * shift);
            leftFront.setVelocity(3000 * lf * shift);
            rightBack.setVelocity(3000 * rb * shift);
            rightFront.setVelocity(3000 * rf * shift);
            telemetry.update();

// Two sensors around an inch apart to tell which way it is going, and go the opposite way. Keep facing any object that was located

        }
    }
}
