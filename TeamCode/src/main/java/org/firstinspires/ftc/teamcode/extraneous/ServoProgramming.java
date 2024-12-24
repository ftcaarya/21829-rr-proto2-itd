package org.firstinspires.ftc.teamcode.extraneous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoProgramming {
    public Servo arm, wrist, rotate, claw;

    public static final double ARM_SERVO_UP = 1.0;
    public static final double ARM_SERVO_DOWN = 0.0;

    public static final double WRIST_SERVO_UP = 1.0;
    public static final double WRIST_SERVO_DOWN = 0.0;

    public static final double ROTATE_SERVO_LEFT = 1.0;
    public static final double ROTATE_SERVO_RIGHT = 0.0;

    public static final double CLAW_OPEN = 1.0;
    public static final double CLAW_CLOSE = 0.0;

    public ServoProgramming(HardwareMap hardwareMap) {
        arm = hardwareMap.get(Servo.class, "arm servo");
        wrist = hardwareMap.get(Servo.class, "wrist servo");
        rotate = hardwareMap.get(Servo.class, "rotate servo");
        claw = hardwareMap.get(Servo.class, "claw servo");
    }


}
