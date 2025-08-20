package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// Generic robot class
public class HackinHoundsHardware extends Hardware {
    public HardwareMap robotMap;

    // Drivetrain Members
    public DcMotorEx  leftFront;
    public DcMotorEx  rightFront;
    public DcMotorEx  leftBack;
    public DcMotorEx  rightBack;

    public DcMotorEx horizontalSlide;
    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;

    //public ServoArm arm;
    public Servo arm;
    public Servo uClaw;
    public Servo lClaw;
    public Servo pitch;
    public Servo wrist;

    public Servo light;
    public Servo light2;



    public AnalogInput rangerLeft;
    public AnalogInput rangerFront;
    public AnalogInput rangerBack;

    public double lastAngle;

    public IMU imu;
    public YawPitchRollAngles angles;

    private YawPitchRollAngles lastAngles;
    private double globalAngle;

    // 1000 ticks was roughly 18 in.
    public static final double TICK_PER_INCH = 432/24;
    public static final double MinPower = 0.1;



    /**
     GLOBAL VARIABLES
     **/

    public double uClaw_Open = 0.69; //0.82
    public double uClaw_Close = 0.65; //0.78

    public double lClaw_Open = 0.1;
    public double lClaw_Close = 0.9;

    public double pitch_Up= 0.506;   //506
    public double pitch_Mid = 0.559; // 0.560
    public double pitch_Close = 0.606;
    public double pitch_Down = 0.616;


    public double wristStraight = 0.32; //0.175
    public double wristMiddle = 0.65;
    public double wristTurned = 0.97; // 0.52

    public double arm_Down = 0.85; // 0.9
    public double arm_Up = 0.25; // 0.25
    public double arm_Slight = 0.74; // 0.78

//    public int rightslideMax = 4100;
//    public int rightslideMin = 0;
//
//    public int leftslideMax = 4100;
//    public int leftslideMin = 0;

//    public int slideUp = 4400;
//    public int slideDown = 0;
//
//    public int horizontalIn = 0;
//    public int horizontalOut = 1400;

    /* Constructor */
    public HackinHoundsHardware(){

    }

    // Override to set actual robot configuration
    public void init(HardwareMap hwMap) {

        // Save reference to Hardware map
        robotMap = hwMap;


        // Define and Initialize Motors for drivetrain
        leftFront  = robotMap.get(DcMotorEx.class, "left_front");
        rightFront = robotMap.get(DcMotorEx.class, "right_front");
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
        leftBack = robotMap.get(DcMotorEx.class, "left_back");
        rightBack = robotMap.get(DcMotorEx.class, "right_back");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        //leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide = robotMap.get(DcMotorEx.class,"leftSlide");
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rotationSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //rotationSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setTargetPosition(0);

        rightSlide = robotMap.get(DcMotorEx.class,"rightSlide");
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rotationSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //rotationSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setTargetPosition(0);



        horizontalSlide = robotMap.get(DcMotorEx.class,"horizontalSlide");
        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        horizontalSlide.setTargetPosition(0);

        //uClaw = new ServoArm(robotMap.get(Servo.class, "uclaw"));
        arm = robotMap.get(Servo.class,"arm");
        uClaw = robotMap.get(Servo.class,"uclaw");

        lClaw =  robotMap.get(Servo.class,"lclaw");
        pitch = robotMap.get(Servo.class,"pitch");
        wrist = robotMap.get(Servo.class,"wrist");

        light = robotMap.get(Servo.class,"light");
        light2 = robotMap.get(Servo.class,"light2");

        rangerLeft = robotMap.get(AnalogInput.class, "rangerLeft");
        rangerFront = robotMap.get(AnalogInput.class, "rangerFront");
        rangerBack = robotMap.get(AnalogInput.class, "rangerBack");



        // Defines the REV Hub's internal IMU (Gyro)
        imu = robotMap.get(IMU.class, "imu");

        //imu.resetYaw();

        Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection usb = Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.LEFT;
        Rev9AxisImuOrientationOnRobot.LogoFacingDirection logo = Rev9AxisImuOrientationOnRobot.LogoFacingDirection.DOWN;

//        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
//        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        Rev9AxisImuOrientationOnRobot orientationOnRobot = new Rev9AxisImuOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        lastAngle = 0;
                //imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double clamp(double x, double min, double max) {
        return Math.max(min,Math.min(max,x));
    }

    //
    public double getAngle() {
        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double deltaAngle = angle - lastAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngle = angle;

        return globalAngle;
    }

    public double getDistance (AnalogInput sensor) {
        return (sensor.getVoltage() * 48.7) - 4.9;
    }
}

