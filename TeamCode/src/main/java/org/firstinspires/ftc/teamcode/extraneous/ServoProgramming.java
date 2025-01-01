package org.firstinspires.ftc.teamcode.extraneous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoProgramming {
    public Servo arm, wrist, rotate, claw, leftArm;

    public static final double ARM_SERVO_DOWN = .582;
    public static final double ARM_SERVO_UP = .45;

    public static final double WRIST_SERVO_DOWN = .84;
    public static final double WRIST_SERVO_UP = 0.375;

    public static final double LEFT_ARM_SERVO_UP = .55;
    public static final double LEFT_ARM_SERVO_DOWN = .378;



    public static final double ROTATE_SERVO_RESET = 0.1;
    public static final double ROTATE_SERVO_PERP = 0.25;
    public static final double ROTATE_SERVO_RIGHT_HALF = 0.19;
    public static final double ROTATE_SERVO_LEFT_HALF = 0.06;



    public static final double CLAW_CLOSE = 1.0;
    public static final double CLAW_OPEN = 0.2;

    public ServoProgramming(HardwareMap hardwareMap) {
        arm = hardwareMap.get(Servo.class, "arm servo");
        leftArm = hardwareMap.get(Servo.class, "left arm servo");
        wrist = hardwareMap.get(Servo.class, "wrist servo");
        rotate = hardwareMap.get(Servo.class, "rotate servo");
        claw = hardwareMap.get(Servo.class, "claw servo");
    }

}
