package org.firstinspires.ftc.teamcode.extraneous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoProgramming {
    public Servo arm, wrist, rotate, claw, leftArm;

//    public static final double ARM_SERVO_DOWN = .15;
//    public static final double ARM_SERVO_UP = 0.05;
//    public static final double ARM_SERVO_SPEC = .17;
//    public static final double ARM_SERVO_SCORE = .27;
    public static final double ARM_SERVO_DOWN = .54;//.57
    public static final double ARM_SERVO_UP = .5;//.45
    public static final double ARM_SERVO_SPEC = .53;//.7
    public static final double ARM_SERVO_SCORE = .835;//.85
    public static final double ARM_ASCENT = .62;


    public static final double WRIST_SERVO_DOWN = .84;
    public static final double WRIST_SERVO_UP = 0.375;
    public static final double WRIST_SERVO_SPEC_SCORE = .2; //.265
    public static final double WRIST_SERVO_SPEC = 0.49;

    public static final double LEFT_ARM_SERVO_UP = .52;//.54
    public static final double LEFT_ARM_SERVO_DOWN = .47;//.44
    public static final double LEFT_ARM_SERVO_SPEC = .49;//.48
    public static final double LEFT_ARM_ASCENT = .5;
    public static final double LEFT_ARM_SERVO_SCORE = .285;//.32



    public static final double ROTATE_SERVO_RESET = 0.1;
    public static final double ROTATE_SERVO_PERP = 0.25;
    public static final double ROTATE_SERVO_RIGHT_HALF = 0.19;
    public static final double ROTATE_SERVO_LEFT_HALF = 0.06;



    public static final double CLAW_CLOSE = 0;
    public static final double CLAW_OPEN = 0.5;

    public ServoProgramming(HardwareMap hardwareMap) {
        arm = hardwareMap.get(Servo.class, "arm servo");
        leftArm = hardwareMap.get(Servo.class, "left arm servo");
        wrist = hardwareMap.get(Servo.class, "wrist servo");
        rotate = hardwareMap.get(Servo.class, "rotate servo");
        claw = hardwareMap.get(Servo.class, "claw servo");
    }

}