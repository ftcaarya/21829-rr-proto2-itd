package org.firstinspires.ftc.teamcode.extraneous;

import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.ARM_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.ARM_SERVO_UP;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.WRIST_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.WRIST_SERVO_UP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Hardware testing", group = "exercises")
public class HardwareTesting extends OpMode {
    ServoProgramming servo;
    AllMech robot;

    @Override
    public void init() {
        servo = new ServoProgramming(hardwareMap);
        robot = new AllMech(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            servo.arm.setPosition(ARM_SERVO_UP);
        }

        if (gamepad1.a) {
            servo.arm.setPosition(ARM_SERVO_DOWN);
        }

        if (gamepad1.x) {
            servo.claw.setPosition(CLAW_OPEN);
        }

        if (gamepad1.b) {
            servo.claw.setPosition(CLAW_CLOSE);
        }

        if (gamepad1.dpad_down) {
            servo.wrist.setPosition(WRIST_SERVO_DOWN);
        }
        if (gamepad1.dpad_up){
            servo.wrist.setPosition(WRIST_SERVO_UP);
        }

    }
}
