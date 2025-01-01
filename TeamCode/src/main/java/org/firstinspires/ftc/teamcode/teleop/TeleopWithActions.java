package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.ARM_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.ARM_SERVO_UP;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.LEFT_ARM_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.LEFT_ARM_SERVO_UP;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.ROTATE_SERVO_LEFT_HALF;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.ROTATE_SERVO_PERP;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.ROTATE_SERVO_RESET;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.ROTATE_SERVO_RIGHT_HALF;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.WRIST_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.WRIST_SERVO_UP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.extraneous.AllMech;
import org.firstinspires.ftc.teamcode.extraneous.ServoProgramming;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Teleop avec l'actions trois", group = "Examples")
public class TeleopWithActions extends OpMode {
    AllMech robot;
    ServoProgramming servo;
    Gamepad currentGamepad1;
    Gamepad currentGamepad2;

    Gamepad previousGamepad1;
    Gamepad previousGamepad2;


    private final FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void init() {
        robot = new AllMech(hardwareMap);
        servo = new ServoProgramming(hardwareMap);

        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();

        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();
    }

    @Override
    public void start() {
        runningActions.add(
                robot.updateLinkPID()
        );
        runningActions.add(
                robot.updateVertPID()
        );
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        TelemetryPacket packet = new TelemetryPacket();

        // update based on gamepads.
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), .5);
        double frontLeftPower = (y + x - rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x + rx) / denominator;


        robot.frontLeft.setPower(frontLeftPower);
        robot.rearLeft.setPower(backLeftPower);
        robot.frontRight.setPower(frontRightPower);
        robot.rearRight.setPower(backRightPower);



        if (gamepad2.x){
        runningActions.add(
                        new InstantAction(() -> servo.rotate.setPosition(ROTATE_SERVO_LEFT_HALF))
        );
        }

        if (gamepad2.b){
            runningActions.add(
                    new InstantAction(() -> servo.rotate.setPosition(ROTATE_SERVO_RIGHT_HALF))

            );
        }

        if (gamepad2.right_stick_button){
            runningActions.add(
                    new InstantAction(() -> servo.rotate.setPosition(ROTATE_SERVO_RESET))

            );
        }
        if (gamepad2.left_stick_button){
            runningActions.add(
                    new InstantAction(() -> servo.rotate.setPosition(ROTATE_SERVO_PERP))

            );
        }


        if (gamepad2.dpad_up) {
            runningActions.add(
                      robot.setElevatorTarget(2500)

            );
        }




        if (gamepad2.dpad_down) {
            runningActions.add(
                            robot.setElevatorTarget(-20)

            );
        }

        if (gamepad2.dpad_left) {
            runningActions.add(

                        robot.setLinkageTarget(500)

            );
        }

        if (gamepad2.dpad_right) {
            runningActions.add(

                            robot.setLinkageTarget(0)
            );
        }

        if (gamepad2.a) {
            runningActions.add(
                    new ParallelAction(
                            new InstantAction(() -> servo.arm.setPosition(ARM_SERVO_DOWN)),
                            new InstantAction(() -> servo.leftArm.setPosition(LEFT_ARM_SERVO_DOWN)),
                            new InstantAction(() -> servo.wrist.setPosition(WRIST_SERVO_DOWN))
                    )
            );
        }



        if (gamepad2.y) {
            runningActions.add(
                    new ParallelAction(
                            new InstantAction(() -> servo.arm.setPosition(ARM_SERVO_UP)),
                            new InstantAction(() -> servo.leftArm.setPosition(LEFT_ARM_SERVO_UP)),
                            new InstantAction(() -> servo.wrist.setPosition(WRIST_SERVO_UP))
                    )
            );
        }

        if (gamepad2.right_bumper) {
            runningActions.add(
                    new InstantAction(() -> servo.claw.setPosition(CLAW_CLOSE))
            );
        }

        if (gamepad2.left_bumper) {
            runningActions.add(
                    new InstantAction(() -> servo.claw.setPosition(CLAW_OPEN))
            );
        }

        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
            runningActions.add(
                            robot.setElevatorTarget(AllMech.elevator.getCurrentPosition() + 100)

            );
        }

        if (!currentGamepad2.left_bumper && previousGamepad2.left_bumper) {
            runningActions.add(

                            robot.setElevatorTarget(AllMech.elevator.getCurrentPosition() - 100)
            );
        }


        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
    }
}
