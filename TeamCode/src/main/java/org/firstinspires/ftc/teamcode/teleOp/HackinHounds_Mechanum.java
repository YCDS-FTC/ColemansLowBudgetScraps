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


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;



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

@TeleOp(name="Mechanum", group="Linear OpMode")
public class HackinHounds_Mechanum extends LinearOpMode {
    // Declare OpMode members. aamir dont screw stuff up
    private ElapsedTime runtime = new ElapsedTime();
    private HackinHoundsHardware robot = new HackinHoundsHardware();

    double p = 0.01, i = 0, d = 0;
    double f = 0.07;

    int target = 0;

    final double ticks_in_degree = 360.0 / 1024.0;
    double shift = 1;

    boolean arrowPressed = false;
    int slidePos = 0;

    boolean LclawOpen = false;
    boolean bumperPressed = false;

    boolean aPressed = false;
    boolean armDown = false;

    boolean xPressed = false;
    boolean pitchDown = false;

    boolean bumperPressed2 = false;
    boolean UclawOpen = false;

    boolean rightBumperPressed = false;

    double clickTime = -10;
    double checkRot = -3;
    double reOpen = -3;

    boolean triggerClicked = false;

    boolean backClicked = false;
    boolean setUpMode = false;

    boolean startPresets = true;

    int atLimit = 1;

    double cycleStart = 0;

    boolean flip = true;

    private final double CAMERA_HEIGHT = 12;      // Camera height in inches above ground/target base
    private final double CAMERA_ANGLE = -28.0;    // Camera tilt angle in degrees (negative if downward)
    private final double CAMERA_X_OFFSET = 0.8;  // Camera ahead of slide base (in)
    private final double TICKS_PER_INCH = 70.27;   // Encoder ticks per inch for slide motor
    private final int MAX_SLIDE_TICKS = 1300;     // Maximum slide extension in ticks
    private final int MIN_SLIDE_TICKS = 6;        // Minimum slide position

    private boolean isTargetLocked = false;
    private int lockedTicks = 0;
    private int lastLockedTicks = 0;
    private int activePipeline = 4;
    private boolean lastValidTarget = false;
    private int validCount = 0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.horizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        telemetry.addData("Status", "Initialized");
        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.update();



        waitForStart();
        runtime.reset();

        //    private boolean rotationAuto = false;
        int horizontalSlidePosition = robot.horizontalSlide.getCurrentPosition();
        robot.horizontalSlide.setTargetPosition(0);
        robot.rightSlide.setTargetPosition(0);
        robot.leftSlide.setTargetPosition(0);
        robot.rightSlide.setTargetPosition(0);
        robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);




//        int rightSlidePosStart = 0;
//        int leftSlidePosStart = 0;
//        int rightDelta = 0;
//        int leftDelta = 0;
//        int targetPos = 0;

//        List<Double> lastScan = new ArrayList<>();
//        lastScan.add(0, 0.0);
//        lastScan.add(1, 0.0);
//        lastScan.add(2, 0.0);
//        lastScan.add(3, 0.0);
//        lastScan.add(4, 0.0);
//        int scan = 0;
//        //double avg;
//        boolean scanning = false;

        double avg = robot.getDistance(robot.rangerLeft);
        double w;

        while (opModeIsActive()) {
            cycleStart = runtime.milliseconds();
            //telemetry.addData("Begin Time", "%f", runtime.milliseconds() - cycleStart);

            if (gamepad1.dpad_up) {
                shift = 1;
            } else if (gamepad1.dpad_right) {
                shift = 0.75;
            } else if (gamepad1.dpad_down) {
                shift = 0.5;
            }
            if (gamepad1.left_stick_button) {
                shift = 1;
            }

            if (gamepad1.right_bumper) {
                if (!rightBumperPressed) {
                    if (shift == 1) {
                        shift = 0.5;
                    } else {
                        shift = 1;
                    }
                }
                rightBumperPressed = true;
            } else {
                rightBumperPressed = false;
            }



            // Send data to Dashboard

            /** DRIVING - Perfectly FINE (don't change) **/

            //telemetry.addData("begining if's finished", "%f", runtime.milliseconds() - cycleStart);

            double facing = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double y = -gamepad1.left_stick_y;
            //double y = 0;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double rotX = x * Math.cos(-facing) - y * Math.sin(-facing);
            rotX = rotX * 1.1;
            double rotY = x * Math.sin(-facing) + y * Math.cos(-facing);

            double d = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(rx), 1);

            double lf = (rotY + rotX + rx) / d;
            double lb = (rotY - rotX + rx) / d;
            double rf = (rotY - rotX - rx) / d;
            double rb = (rotY + rotX - rx) / d;

            robot.leftBack.setVelocity(3000 * lb * shift);
            robot.leftFront.setVelocity(3000 * lf * shift);
            robot.rightBack.setVelocity(3000 * rb * shift);
            robot.rightFront.setVelocity(3000 * rf * shift);

            //telemetry.addData("Driving Finished", "%f", runtime.milliseconds() - cycleStart);

            /** LIMELIGHT WOP MIGHT CHANGE NOT STABLE**/

//            For when you use the lights:
//             - 0.279: RED
//             - 0.388: YELLOW
//             - 0.611: BLUE
//             - 0.500: GREEN
//

            /** Pipeline 0: yellow detection
             Pipeline 1: blue detection
             Pipeline 2: red detection


            // Limelight status telemetry
            LLStatus status = robot.limelight.getStatus();
            telemetry.addData("Name", "%s", status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            // Reset lock
//            if (gamepad1.a) {
//                isTargetLocked = false;
//                robot.limelight.setPollRateHz(5);
//                lockedTicks = 0;
//                robot.horizontalSlide.setTargetPosition(6);
//                robot.horizontalSlide.setPower(0.5);
//                telemetry.addData("Status", "Target lock reset");
//            }

            // Declare trueAngle here so it’s in scope for telemetry
            double trueAngle = 0.0;
            double ty = 0.0; // For telemetry use outside the block

            // Process Limelight data if not locked
            if (!isTargetLocked && gamepad1.a) {
                LLResult result = robot.limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    ty = result.getTy();
                    trueAngle = ty - CAMERA_ANGLE;
                    double theta = Math.toRadians(Math.abs(trueAngle));

                    double distance;
                    if (Math.abs(trueAngle) < 5.0 || Math.abs(trueAngle) > 85.0) {
                        distance = 10;
                        telemetry.addData("Warning", "Angle out of range, using max distance");
                    } else {
                        double cameraToTargetDistance = CAMERA_HEIGHT / Math.tan(theta);
                        distance = cameraToTargetDistance + CAMERA_X_OFFSET;
                        if (distance < 0){
                            distance = 0;
                            telemetry.addData("Error", "Calculated distance negative");
                        }
                    }

                    lockedTicks = (int) (distance * TICKS_PER_INCH);
                    lockedTicks = Math.max(MIN_SLIDE_TICKS, Math.min(lockedTicks, MAX_SLIDE_TICKS));

                    isTargetLocked = true;
                    robot.limelight.setPollRateHz(0);

                    telemetry.addData("Target Locked", "Distance: %.2f in, Ticks: %d", distance, lockedTicks);
                } else {
                    telemetry.addData("Limelight", "No valid target...");
                }
            }

            // Command slide motor
            if(lockedTicks != lastLockedTicks){
                robot.horizontalSlide.setTargetPosition(lockedTicks);
                lastLockedTicks = lockedTicks;
            }
            if (!robot.horizontalSlide.isBusy()) {
                robot.horizontalSlide.setPower(0); // Stop when reached
            } else {
                robot.horizontalSlide.setPower(0.5); // Move if busy
            }

            LLResult result = robot.limelight.getLatestResult();

            if(activePipeline == 1 && result.isValid()){ //blue
                robot.light.setPosition(0.611);
                robot.light2.setPosition(0.611);
            } else if (activePipeline == 2 && result.isValid()){ //red
                robot.light.setPosition(0.279);
                robot.light2.setPosition(0.279);
            } else if (activePipeline == 3 && result.isValid()){ //yellow
                robot.light.setPosition(0.388);
                robot.light2.setPosition(0.388);
            } else{
                robot.light.setPosition(0);
                robot.light2.setPosition(0);
            }


//            boolean validTarget = result != null && result.isValid();
//            if (validTarget) {
//                    resetLeds();
//                    if (activePipeline == 1) { // Blue
//                        robot.light.setPosition(0.611);
//                        robot.light2.setPosition(0.611);
//                    } else if (activePipeline == 0) { // Yellow
//                        robot.light.setPosition(0.388);
//                        robot.light2.setPosition(0.388);
//                    } else if (activePipeline == 2) { // Red
//                        robot.light.setPosition(0.279);
//                        robot.light2.setPosition(0.279);
//                    }
//
//            } else {
//                resetLeds();
//            }



            // LED control based on valid target and pipeline
// Additional telemetry if result is valid
            if (result != null && result.isValid()) {
                telemetry.addData("tx", result.getTx());
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("tync", result.getTyNC());
                telemetry.addData("True Angle (deg)", trueAngle); // Now in scope
            }
            telemetry.addData("lockedTicks", lockedTicks);
            telemetry.addData("Motor Pos", robot.horizontalSlide.getCurrentPosition());
            telemetry.addData("isBusy", robot.horizontalSlide.isBusy());
            telemetry.addData("Motor Power", robot.horizontalSlide.getPower());
            telemetry.addData("Light", robot.light.getPosition());
            telemetry.addData("Light2", robot.light2.getPosition());


            /** Vertical Slides - Needs to be changed into 3 presets on 2 buttons **/

            if (gamepad2.dpad_up) {
                if (!arrowPressed) {
                    if (slidePos == 0) {
                        //targetPos = 1800;
                        //moveSlidesUpToTargetPos(targetPos);
                        robot.rightSlide.setTargetPosition(1800);
                        robot.leftSlide.setTargetPosition(1800);
                        slidePos = 1;
                    } else if (slidePos == 1) {
                        //targetPos = 3800;
                        //moveSlidesUpToTargetPos(targetPos);
                        robot.rightSlide.setTargetPosition(3800);
                        robot.leftSlide.setTargetPosition(3800);
                        slidePos = 2;
                    }
                }
                arrowPressed = true;
            } else if (gamepad2.dpad_down) {
                if (!arrowPressed) {
                    if (slidePos == 1) {
                        //moveSlidesDownToTargetPos(0);
                        robot.rightSlide.setTargetPosition(0);
                        robot.leftSlide.setTargetPosition(0);
//                        robot.arm.getController().pwmDisable();
                        slidePos = 0;
                    } else if (slidePos == 2) {
                        //moveSlidesDownToTargetPos(1800);
                        robot.rightSlide.setTargetPosition(1800);
                        robot.leftSlide.setTargetPosition(1800);
                        slidePos = 1;
                    }
                }
                arrowPressed = true;
            } else {
                arrowPressed = false;
                if (robot.leftSlide.getTargetPosition() == 0 && robot.leftSlide.getCurrentPosition() < 25) {
                    telemetry.addLine("Slides Power: Off");
                    robot.leftSlide.setPower(0);
                    robot.rightSlide.setPower(0);
                } else {
                    if (robot.leftSlide.getCurrentPosition() <= robot.leftSlide.getTargetPosition() + 10 && robot.leftSlide.getCurrentPosition() >= robot.leftSlide.getTargetPosition() - 10) {
                        telemetry.addLine("Slides Power: Limied");
                        robot.leftSlide.setPower(0.3);
                        robot.rightSlide.setPower(0.3);
                    } else {
                        telemetry.addLine("Slides Power: Full");
                        robot.leftSlide.setPower(1);
                        robot.rightSlide.setPower(1);
                    }
                }
            }

            if (gamepad1.left_bumper ) {
                robot.arm.getController().pwmDisable();
            }

            //telemetry.addData("Vertical Slides Finished", "%f", runtime.milliseconds() - cycleStart);

//            int leftSlidePosition = robot.leftSlide.getCurrentPosition();
//            int rightSlidePosition = robot.rightSlide.getCurrentPosition();
//
//            int js = -gamepad2.right_stick_y;
//
//            if (atLimit == 2) {
//                js = robot.clamp(js, -1, 0);
//            } else if (atLimit == 0) {
//                js = robot.clamp(js, 0, 1);
//            }
//
//            if (Math.abs(js) > 0.1) {
//                robot.rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//                robot.rightSlide.setPower(js);
//                robot.leftSlide.setPower(js);
//                robot.leftSlide.setTargetPosition(leftSlidePosition);
//                robot.rightSlide.setTargetPosition(rightSlidePosition);
//            } else {
//                robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.rightSlide.setPower(0.5);
//                robot.leftSlide.setPower(0.5);
//            }
//
//            if (robot.rightSlide.getTargetPosition() > 4000) {
//                if (atLimit != 2) {
//                    robot.rightSlide.setTargetPosition(4100);
//                    robot.leftSlide.setTargetPosition(4100);
//                }
//                atLimit = 2;
//            } else if (robot.rightSlide.getTargetPosition() < 100) {
//                if (atLimit != 0) {
//                    robot.rightSlide.setTargetPosition(0);
//                    robot.leftSlide.setTargetPosition(0);
//                }
//                atLimit = 0;
//            } else {
//                atLimit = 1;
//            }


            /** Horizontal Slides - Works well but might need a bit of tweaking **/
            horizontalSlidePosition = robot.horizontalSlide.getCurrentPosition();
            if (setUpMode) {
                robot.horizontalSlide.setPower(-gamepad2.left_stick_y);
            }
//            else {
//                if (Math.abs(-gamepad2.left_stick_y) > 0.1) {
//                    robot.horizontalSlide.setTargetPosition(horizontalSlidePosition + (int) (-gamepad2.left_stick_y * 150));
//                    if (robot.horizontalSlide.getTargetPosition() > 1300) {
//                        robot.horizontalSlide.setTargetPosition(1300);
//                    } else if (robot.horizontalSlide.getTargetPosition() < 0) {
//                        robot.horizontalSlide.setTargetPosition(5);
//                    }
//                } else {
//                    if (robot.horizontalSlide.getTargetPosition() == 0 && robot.horizontalSlide.getCurrentPosition() < 20) {
//                        robot.horizontalSlide.setPower(0);
//                    } else {
//                        robot.horizontalSlide.setPower(0.5);
//                    }
//                }
//            }

            if (setUpMode) {
                robot.horizontalSlide.setPower(-gamepad2.left_stick_y);
            } else {
                if(Math.abs(-gamepad2.left_stick_y) > 0.1){
                    target += (int) (-gamepad2.left_stick_y * 100);
                    target = Math.max(0, Math.min(target, 1500));
                }
            }


            int slidePos = robot.horizontalSlide.getCurrentPosition();

            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f ;




            //telemetry.addData("Horizontal Slides Finished", "%f", runtime.milliseconds() - cycleStart);

            if (gamepad2.dpad_left) {
                if (robot.wrist.getPosition() >= robot.wristStraight - 0.3 && robot.wrist.getPosition() <= robot.wristStraight + 0.3) {
                    target = 0;
                    robot.pitch.setPosition(robot.pitch_Mid);
                    robot.wrist.setPosition(robot.wristStraight);
                    pitchDown = true;
                    isTargetLocked = false;
                    lockedTicks = 0;
                    telemetry.addData("Status", "Target lock reset");
                } else {
                    checkRot = runtime.seconds();
                }
            }

            if ((runtime.seconds() - checkRot) < 0.3) {
                robot.pitch.setPosition(robot.pitch_Close);
                robot.wrist.setPosition(robot.wristStraight);
            } else if ((runtime.seconds() - checkRot) < 0.5) {
                robot.horizontalSlide.setTargetPosition(0);
                robot.pitch.setPosition(robot.pitch_Mid);
                robot.wrist.setPosition(robot.wristStraight);
                pitchDown = true;
            }

            //telemetry.addData("Horza Slide Reset finished", "%f", runtime.milliseconds() - cycleStart);

            /** Wrist - Seems to be working all fine and dandy **/

            if (setUpMode) {
                robot.rightSlide.setPower(gamepad2.right_stick_y);
                robot.leftSlide.setPower(gamepad2.right_stick_y);
            } else {
                robot.wrist.setPosition(robot.wrist.getPosition() - (gamepad2.right_stick_y * 0.05));
            }

            if (gamepad2.right_trigger > 0.5) {
                if (!triggerClicked) {
                    robot.wrist.setPosition(robot.wrist.getPosition() - 0.1);
                }
                triggerClicked = true;
            } else if (gamepad2.left_trigger > 0.5) {
                if (!triggerClicked) {
                    robot.wrist.setPosition(robot.wrist.getPosition() + 0.1);
                }
                triggerClicked = true;
            } else {
                triggerClicked = false;
            }

            //telemetry.addData("Wrist Settings finished", "%f", runtime.milliseconds() - cycleStart);


            /** Arm (Button A) - Seems to be working all fine and dandy **/

            if (gamepad2.a) {
                if (!aPressed) {
                    if (armDown) {
                        robot.arm.setPosition(robot.arm_Up);
                        armDown = false;
                    } else {
                        robot.arm.setPosition(robot.arm_Down);
                        armDown = true;
                    }
                }
                aPressed = true;
            } else {
                aPressed = false;
            }

            if (gamepad1.right_trigger > 0.01){
                robot.arm.setPosition(0.2);
            }


//            if (gamepad1.right_trigger > 0.1) {
//                    robot.arm.setPosition(robot.arm.getPosition() - 0.1);
//            }


            telemetry.addData("A press finished", "%f", runtime.milliseconds() - cycleStart);

            /** Pitch (Button X) - Seems to be working all fine and dandy **/

            if (gamepad2.x) {
                if (!xPressed) {
                    if (robot.pitch.getPosition() < 0.57) {
                        robot.wrist.setPosition(robot.wristStraight);
                        reOpen = runtime.seconds();
                    }
//                    if (pitchDown) {
//                        robot.pitch.setPosition(robot.pitch_Close);
                        //reOpen = runtime.seconds();
//                        pitchDown = false;
//                    } else {
                    robot.pitch.setPosition(robot.pitch_Down);
//                        pitchDown = true;
//                    }
                }
                xPressed = true;
            } else {
                xPressed = false;
            }

            if ((runtime.seconds() - reOpen) < 0.45) {
                robot.lClaw.setPosition(robot.lClaw_Close);
                LclawOpen = false;
            } else if ((runtime.seconds() - reOpen) < 0.5) {
                robot.lClaw.setPosition(robot.lClaw_Open);
                LclawOpen = true;
            }




            //telemetry.addData("X press finished", "%f", runtime.milliseconds() - cycleStart);

            /** Claw (Left Bumper) - Seems to be working all fine and dandy **/

            if (gamepad2.left_bumper) {
                if (!bumperPressed2) {
                    if (UclawOpen) {
                        robot.uClaw.setPosition(robot.uClaw_Close);
                        UclawOpen = false;
                    } else {
                        robot.uClaw.setPosition(robot.uClaw_Open);
                        UclawOpen = true;
                    }
                }
                bumperPressed2 = true;
            } else {
                bumperPressed2 = false;
            }

            //telemetry.addData("Left bump press finished", "%f", runtime.milliseconds() - cycleStart);


            /** Claw (Right Bumper) - Seems to be working all fine and dandy **/

            if (gamepad2.right_bumper) {
                if (!bumperPressed) {
                    if (LclawOpen) {
                        robot.lClaw.setPosition(robot.lClaw_Close);
                        LclawOpen = false;
                    } else {
                        robot.lClaw.setPosition(robot.lClaw_Open);
                        LclawOpen = true;
                    }
                }
                bumperPressed = true;
            } else {
                bumperPressed = false;
            }

            //telemetry.addData("Right bump press finished", "%f", runtime.milliseconds() - cycleStart);


            /** Cycle multi STEP PRESET**/


            if (gamepad2.b) {
                clickTime = runtime.seconds();
                flip = true;
            } else if (gamepad2.dpad_right) {
                clickTime = runtime.seconds();
                flip = false;
            }

            if ((runtime.seconds() - clickTime) < 0.5) {
                robot.pitch.setPosition(robot.pitch_Up);
                pitchDown = true;
                if (flip) { robot.wrist.setPosition(robot.wristTurned); }
            } else if ((runtime.seconds() - clickTime) < 0.9) {
                robot.uClaw.setPosition(robot.uClaw_Close);
                UclawOpen = false;
            } else if ((runtime.seconds() - clickTime) < 1.2) {
                robot.lClaw.setPosition(robot.lClaw_Open);
                LclawOpen = false;
            } else if ((runtime.seconds() - clickTime) < 1.6) {
                robot.arm.setPosition(robot.arm_Slight);
                armDown = true;
            }

            //telemetry.addData("Transfer Finished", "%f", runtime.milliseconds() - cycleStart);


            if (gamepad2.y) {
                robot.rightSlide.setTargetPosition(0);
                robot.leftSlide.setTargetPosition(0);
                slidePos = 0;
                robot.pitch.setPosition(robot.pitch_Mid);
                robot.wrist.setPosition(robot.wristStraight);
                robot.arm.setPosition(robot.arm_Down);
                robot.uClaw.setPosition(robot.uClaw_Open);
                robot.lClaw.setPosition(robot.lClaw_Open);
                armDown = true;
                UclawOpen = true;
                LclawOpen = false;
                pitchDown = true;
                clickTime = -10;
            }

            if (gamepad2.back) {
                if (!backClicked) {
                    if (setUpMode) {
                        robot.light.setPosition(0);
                        robot.light2.setPosition(0);
                        setUpMode = false;
                        robot.horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.horizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    } else {
                        robot.light.setPosition(0.333);
                        robot.light2.setPosition(0.333);
                        setUpMode = true;
                        robot.horizontalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                }
                backClicked = true;
            } else {
                backClicked = false;
            }

            telemetry.addData("before telemetry", "%f", runtime.milliseconds() - cycleStart);

            if (gamepad1.back) {
                robot.imu.resetYaw();
            }


            /** Drive Motors **/
//            telemetry.addLine("Drive Motors Current Pos:");
//            telemetry.addData("Back:", "%d / %d", robot.leftBack.getCurrentPosition(), robot.rightBack.getCurrentPosition());
//            telemetry.addData("Front:", "%d / %d", robot.leftFront.getCurrentPosition(), robot.rightFront.getCurrentPosition());
//
//            telemetry.addLine("Drive Motors Currents:");
//            telemetry.addData("Front:", "%f / %f", robot.leftFront.getCurrent(CurrentUnit.MILLIAMPS), robot.rightFront.getCurrent(CurrentUnit.MILLIAMPS));
//            telemetry.addData("Back:", "%f / %f", robot.leftBack.getCurrent(CurrentUnit.MILLIAMPS), robot.rightBack.getCurrent(CurrentUnit.MILLIAMPS));


            /** Slides **/
//            telemetry.addData("Left Slide Cur/Tar:", "%d / %d", robot.leftSlide.getCurrentPosition(), robot.leftSlide.getTargetPosition());
//            telemetry.addData("Right Slide Cur/Tar:", "%d / %d", robot.rightSlide.getCurrentPosition(), robot.rightSlide.getTargetPosition());
//
            telemetry.addData("Horizontal Slide:", "%d", robot.horizontalSlide.getCurrentPosition());
            telemetry.addData("IMU HEADING:", "%f", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//            telemetry.addData("Degree testing:", "%f", robot.getAngle());


            /** Servos **/
            telemetry.addData("armPos", "%f", robot.arm.getPosition());
//            telemetry.addData("clawPos", "%f", robot.lClaw.getPosition());
            telemetry.addData("pitchPos", "%f", robot.pitch.getPosition());
//            telemetry.addData("clawPos2", "%f", robot.uClaw.getPosition());
            telemetry.addData("wristPos", "%f", robot.wrist.getPosition());


//            lastScan.set(scan, robot.getDistance(robot.rangerLeft));
//            if (scan == 4) {
//                scan = 0;
//                scanning = true;
//            } else {
//                scan++;
//            }
//            if (scanning) {
//                avg = (lastScan.get(0) + lastScan.get(1) + lastScan.get(2) + lastScan.get(3) + lastScan.get(4)) / 5;
//                telemetry.addData("Ranger Left avg: ", "%f", avg);
//            }

            w = 0.2;
            avg = w*robot.getDistance(robot.rangerLeft) + (1-w)*avg;
            telemetry.addData("Ranger Left filter: ", "%f", avg);
            telemetry.addData("Ranger Left: ", "%f", robot.getDistance(robot.rangerLeft));
            //telemetry.addData("Runtime:", "%f", runtime.seconds());

            telemetry.addData("Telemtry finished", "%f", runtime.milliseconds() - cycleStart);
            telemetry.update();
        }
    }

    private void resetLeds() {
        robot.light.setPosition(0);
        robot.light2.setPosition(0);
    }

//    public void moveSlidesUpToTargetPos(int _targetPos) {
//        int rightSlidePosStart = robot.rightSlide.getCurrentPosition();
//        int leftSlidePosStart = robot.leftSlide.getCurrentPosition();
//        int rightDelta = _targetPos - rightSlidePosStart;
//        int leftDelta = _targetPos - leftSlidePosStart;
//        while (rightDelta >= 10) { /* TO-DO: set tolerance level here for how close each slide should be */
//
//            if (rightDelta >= 100) /* if delta is greater than 100, increment position by 100 */
//                robot.rightSlide.setTargetPosition(robot.rightSlide.getCurrentPosition() + 100);
//            else /* else increment by delta */
//                robot.rightSlide.setTargetPosition(robot.rightSlide.getCurrentPosition() + rightDelta);
//
//            sleep(1000); /* TO-DO: set time here that works for the increment amount */
//            rightDelta = _targetPos - robot.rightSlide.getCurrentPosition();
//        }
//        while (leftDelta >= 10) {
//            if (leftDelta >= 100) /* if delta is greater than 100, increment position by 100 */
//                robot.leftSlide.setTargetPosition(robot.leftSlide.getCurrentPosition() + 100);
//            else /* else increment by delta */
//                robot.leftSlide.setTargetPosition(robot.leftSlide.getCurrentPosition() + leftDelta);
//
//            sleep(1000);
//            leftDelta = _targetPos - robot.leftSlide.getCurrentPosition();
//        }
//    }
//
//    public void moveSlidesDownToTargetPos(int _targetPos) {}

}

