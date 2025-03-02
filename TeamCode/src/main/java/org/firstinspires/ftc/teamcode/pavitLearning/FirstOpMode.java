package org.firstinspires.ftc.teamcode.pavitLearning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Pavit Testing", group = "exercise")
public class FirstOpMode extends OpMode {

    public DcMotor testMotor;

    @Override
    public void init() {

        testMotor = hardwareMap.get(DcMotor.class, "elevator");

    }

    @Override
    public void loop() {

        if (-gamepad1.right_stick_y > 0) {
            testMotor.setPower(0.5);
        } else if (-gamepad1.right_stick_y < 0) {
            testMotor.setPower(-0.5);
        }


    }
}
