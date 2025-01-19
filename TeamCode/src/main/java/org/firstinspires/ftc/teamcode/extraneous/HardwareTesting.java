package org.firstinspires.ftc.teamcode.extraneous;

import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.ARM_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.ARM_SERVO_UP;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.WRIST_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.WRIST_SERVO_UP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Hardware testing", group = "exercises")
public class HardwareTesting extends OpMode {
    ServoProgramming servo;
    AllMech robot;

    Servo servoTesting;

    @Override
    public void init() {
        servo = new ServoProgramming(hardwareMap);
        robot = new AllMech(hardwareMap);

        servoTesting = hardwareMap.get(Servo.class, "testing");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            servoTesting.setPosition(1);
        } else if (gamepad1.b) {
            servoTesting.setPosition(0);
        }

        telemetry.addData("position: ", servoTesting.getPosition());
    }
}
