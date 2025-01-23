package org.firstinspires.ftc.teamcode.extraneous;

import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.ARM_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.ARM_SERVO_UP;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.WRIST_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.WRIST_SERVO_UP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier;
import dev.frozenmilk.dairy.pasteurized.Pasteurized;

@TeleOp(name = "Hardware testing", group = "exercises")
public class HardwareTesting extends OpMode {
    ServoProgramming servo;
    AllMech robot;

    MecanumDrive drive;
    Servo servoTesting;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private int specCounter = -1;
    private int sampleCounter = -1;

    @Override
    public void init() {
        servo = new ServoProgramming(hardwareMap);
        robot = new AllMech(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        servoTesting = hardwareMap.get(Servo.class, "testing");


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

        EnhancedBooleanSupplier aButton = Pasteurized.gamepad1().a();
        EnhancedBooleanSupplier bButton = Pasteurized.gamepad1().b();
        EnhancedBooleanSupplier xButton = Pasteurized.gamepad1().x();
        EnhancedBooleanSupplier rightBumper = Pasteurized.gamepad1().rightBumper();
        EnhancedBooleanSupplier leftBumper = Pasteurized.gamepad1().leftBumper();

        TelemetryPacket packet = new TelemetryPacket();

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));

        drive.updatePoseEstimate();

        if (aButton.onTrue()) {
            specCounter++;
        }

        if (xButton.onTrue()) {
            sampleCounter++;
        }

        if (specCounter > 3) {
            specCounter = 0;
        }

        if (sampleCounter > 4) {
            sampleCounter = 0;
        }

        if (bButton.state()) {
            runningActions.add(new ParallelAction(
                    robot.setElevatorTarget(-20),
                    robot.setLinkageTarget(100),
                    robot.servoDown(),
                    robot.clawOpen()
            ));
        }

        if (sampleCounter == 0) {
            runningActions.add(
                    new SequentialAction(
                            new ParallelAction(
                                    robot.servoDown(),
                                    robot.clawOpen()
                            ),
                            robot.setElevatorTarget(-20),
                            robot.setLinkageTarget(-550)
                    )
            );
        }

        if (sampleCounter == 1) {
            runningActions.add(
                    new SequentialAction(
                            robot.setElevatorTarget(300)
                    )
            );
        }

        if (sampleCounter == 2) {
            runningActions.add(
                    new SequentialAction(
                            robot.setElevatorTarget(-20)
                    )
            );
        }

        if (sampleCounter == 3) {
            runningActions.add(
                    new SequentialAction(
                            robot.setLinkageTarget(100)
                    )
            );
        }

        if (sampleCounter == 4) {
            runningActions.add(
                    new SequentialAction(
                            robot.setElevatorTarget(2500),
                            robot.servoUp(),
                            new SleepAction(0.4),
                            robot.clawOpen()
                    )
            );
        }


        if (rightBumper.state()) {
            runningActions.add(
                    new SequentialAction(
                            robot.servoGet(),
                            new SleepAction(0.3),
                            robot.clawClose(),
                            new SleepAction(0.2),
                            robot.servoDown()
                    )
            );
        }

        if (specCounter == 0) {
            runningActions.add(
                    new SequentialAction(
                            robot.setElevatorTarget(-20),
                            new ParallelAction(
                                    robot.servoSpecimen(),
                                    robot.clawOpen()
                            ),
                        robot.setLinkageTarget(-550)
            ));
        }

        if (specCounter == 1) {
            runningActions.add(
                    new SequentialAction(
                        robot.clawClose(),
                        new SleepAction(0.2),
                        new ParallelAction(
                                robot.servoUp(),
                                robot.setLinkageTarget(100)
                        )
            ));
        }

        if (specCounter == 2) {
            runningActions.add(
                    new ParallelAction(
                            robot.servoSpecimenScore(),
                            robot.setElevatorTarget(1000)
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


        // update running actions
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
