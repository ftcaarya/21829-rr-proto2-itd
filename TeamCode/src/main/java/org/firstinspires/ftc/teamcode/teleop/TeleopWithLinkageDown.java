package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.extraneous.AllMech;
import org.firstinspires.ftc.teamcode.extraneous.ServoProgramming;

import java.util.ArrayList;
import java.util.List;

import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier;
import dev.frozenmilk.dairy.pasteurized.Pasteurized;

@TeleOp(name = "Teleop avec l'actions trois Linkage Down", group = "Examples")
public class TeleopWithLinkageDown extends OpMode {
    AllMech robot;
    ServoProgramming servo;
    Gamepad currentGamepad1;
    Gamepad currentGamepad2;

    Gamepad previousGamepad1;
    Gamepad previousGamepad2;

    MecanumDrive drive;

    public int clawCounter = -1;
    private int specCounter = -1;


    private final FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void init() {
        robot = new AllMech(hardwareMap);
        servo = new ServoProgramming(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

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

        runningActions.add(
                new SequentialAction(
                        robot.setElevatorTarget(0),
                        robot.servoDown(),
                        robot.setLinkageTarget(450)
                )

        );
    }

    @Override
    public void loop() {
        EnhancedBooleanSupplier clawButton = Pasteurized.gamepad2().rightBumper();
        EnhancedBooleanSupplier specButton = Pasteurized.gamepad2().rightStickButton();

        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        TelemetryPacket packet = new TelemetryPacket();

        // update based on gamepads.
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

//        if (gamepad1.options) {
//            robot.imu.resetYaw();
//        }
//
//        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

//        double frontLeftPower = (rotY + rotX + rx);
//        double backLeftPower = (rotY - rotX + rx);
//        double frontRightPower = (rotY - rotX - rx);3
//        double backRightPower = (rotY + rotX - rx);

//        double frontLeftPower = (y + x + rx);
//        double backLeftPower = (y - x + rx);
//        double frontRightPower = (y - x - rx);
//        double backRightPower = (y + x - rx);
//
//        frontLeftPower = Range.clip(frontLeftPower, -0.7, 0.7);
//        backLeftPower = Range.clip(backLeftPower, -0.7, 0.7);
//        frontRightPower = Range.clip(frontRightPower, -0.7, 0.7);
//        backRightPower = Range.clip(backRightPower, -0.7, 0.7);
//
//        robot.frontLeft.setPower(frontLeftPower);
//        robot.rearLeft.setPower(backLeftPower);
//        robot.frontRight.setPower(frontRightPower);
//        robot.rearRight.setPower(backRightPower);

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        (-gamepad1.left_stick_y * 0.8),
                        (-gamepad1.left_stick_x * 0.8)
                ),
                -gamepad1.right_stick_x
        ));

//        if (gamepad1.left_stick_button) {
//            runningActions.add(
//                    robot.setHangingTarget(2500)
//            );
//        }
//
//        if (gamepad1.right_stick_button) {
//            runningActions.add(
//                    robot.setHangingTarget(0)
//            );
//        }

        drive.updatePoseEstimate();

        if (specButton.onTrue()) {
            specCounter++;
        }

        if (specCounter > 3) {
            specCounter = 0;
        }

        if (specCounter == 0) {
            runningActions.add(
                    new SequentialAction(
                            robot.setElevatorTarget(0),
                            new ParallelAction(
                                    robot.servoSpecimen(),
                                    robot.clawOpen()
                            ),
                            robot.setLinkageTarget(0)
                    ));
        }

        if (specCounter == 1) {
            runningActions.add(
                    new SequentialAction(
                            robot.clawClose(),
                            new SleepAction(0.2),
                            new ParallelAction(
                                    robot.servoUp(),
                                    robot.setLinkageTarget(550)
                            )
                    ));
        }

        if (specCounter == 2) {
            runningActions.add(
                    new ParallelAction(
                            robot.servoSpecimenScore(),
                            robot.setElevatorTarget(400)
                    )
            );
        }

        if (specCounter == 3) {
            runningActions.add(
                    new SequentialAction(
                            robot.setElevatorTarget(1100),
                            new SleepAction(0.2),
                            robot.clawOpen()
                    )
            );
        }

        if (currentGamepad2.x && !previousGamepad2.x){
            runningActions.add(
                    new InstantAction(() -> servo.rotate.setPosition(servo.rotate.getPosition() + .1))
            );
        }

        if (!currentGamepad2.b && previousGamepad2.b){
            runningActions.add(
                    new InstantAction(() -> servo.rotate.setPosition(servo.rotate.getPosition() - .1))

            );
        }


        if (gamepad2.dpad_up) {
            runningActions.add(
                    robot.setElevatorTarget(2500)

            );
        }

        if (gamepad2.dpad_down) {
            runningActions.add(
                    new SequentialAction(
                            robot.servoDown(),
                            new SleepAction(1),
                            robot.setElevatorTarget(0)
                    )

            );
        }

        if (gamepad2.dpad_left) {
            runningActions.add(
                    new ParallelAction(
                            robot.servoDown(),
                            robot.setLinkageTarget(400)
                    )


            );
        }

        if (gamepad2.dpad_right) {
            runningActions.add(
                    robot.setLinkageTarget(0)
            );
        }

        if (gamepad2.a) {
            runningActions.add(
                    robot.servoDown()
            );
        }



        if (gamepad2.y) {
            runningActions.add(
                    robot.servoUp()
            );
        }

        if (clawButton.onTrue()) {
            clawCounter++;
        }

        if (clawCounter > 1) {
            clawCounter = 0;
        }

        if (clawCounter == 0) {
            runningActions.add(
                    robot.clawOpen()
            );
        }

        if (clawCounter == 1) {
            runningActions.add(
                    robot.clawClose()
            );
        }

        if (gamepad2.right_bumper) {
            runningActions.add(
                    robot.clawClose()
            );
        }

        if (gamepad2.left_bumper) {
            runningActions.add(
                    robot.clawOpen()
            );
        }

        if (gamepad1.right_bumper) {
            runningActions.add(
                    robot.setElevatorTarget(800)

            );
        }

        if (!currentGamepad1.left_bumper && previousGamepad1.left_bumper) {
            runningActions.add(
                    robot.setElevatorTarget(AllMech.elevator.getCurrentPosition() - 200)
            );
        }

        if (!currentGamepad1.y && previousGamepad1.y) {
            runningActions.add(
                    new ParallelAction(
                            new InstantAction(() -> servo.arm.setPosition(servo.arm.getPosition() - 0.05)),
                            new InstantAction(() -> servo.leftArm.setPosition(servo.leftArm.getPosition() + 0.019))
                    )

            );
        }

        if (!currentGamepad1.left_bumper && previousGamepad1.left_bumper) {
            runningActions.add(
                    robot.setElevatorTarget(AllMech.elevator.getCurrentPosition() - 200)
            );
        }

        if (currentGamepad1.a) {
            runningActions.add(
                    new SequentialAction(
                            robot.clawOpen(),
                            robot.servoGet(),
                            new SleepAction(0.15),
                            robot.clawClose()
                    )

            );
        }

        if (gamepad1.dpad_up){
            runningActions.add(
                    robot.setElevatorTarget(1100)
            );
        }

        if (gamepad1.dpad_down){
            runningActions.add(
                    robot.servoSpecimen()
            );
        }

        if (gamepad1.x){
            runningActions.add(
                    new SequentialAction(
                            robot.setElevatorTarget(200)

                    )

            );
        }
        if (gamepad1.b){
            runningActions.add(
                    new ParallelAction(
                            robot.servoSpecimenScore()
                    )

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