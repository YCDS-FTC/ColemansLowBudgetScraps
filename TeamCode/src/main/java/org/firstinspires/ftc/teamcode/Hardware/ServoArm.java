package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
@Config

public class ServoArm {
    public static double POS_OFFSET = 0;
    private Servo servo;

    public ServoArm(Servo servo) {
        this.servo = servo;
    }

    public void setPosition(double pos) {
        servo.setPosition(POS_OFFSET + pos);
    }

    public double getPosition() {
        return(servo.getPosition());
    }
}
