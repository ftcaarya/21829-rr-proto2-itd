package org.firstinspires.ftc.teamcode.teleop;

import static org.slf4j.MDC.put;

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

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.extraneous.AllMech;
import org.firstinspires.ftc.teamcode.extraneous.ServoProgramming;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier;
import dev.frozenmilk.dairy.core.util.supplier.numeric.EnhancedDoubleSupplier;
import dev.frozenmilk.dairy.pasteurized.Pasteurized;
import dev.frozenmilk.dairy.pasteurized.PasteurizedGamepad;
import dev.frozenmilk.dairy.pasteurized.SDKGamepad;
import dev.frozenmilk.dairy.pasteurized.layering.LayeredGamepad;
import dev.frozenmilk.dairy.pasteurized.layering.MapLayeringSystem;

public class LayeringTest  extends OpMode {
    enum Layers {
        SAMPLES,
        SPECIMEN
    }

    SDKGamepad sampleGamepad = new SDKGamepad(gamepad2);
    SDKGamepad specimenGamepad = new SDKGamepad(gamepad2);

    Map<Layers, PasteurizedGamepad<EnhancedDoubleSupplier, EnhancedBooleanSupplier>> pasteurizedGamepadMap = new
            HashMap<Layers, PasteurizedGamepad<EnhancedDoubleSupplier, EnhancedBooleanSupplier>>(){{
        put(Layers.SAMPLES, sampleGamepad);
        put(Layers.SPECIMEN, specimenGamepad);
    }};
    MapLayeringSystem<Layers, EnhancedDoubleSupplier, EnhancedBooleanSupplier,
            PasteurizedGamepad<EnhancedDoubleSupplier, EnhancedBooleanSupplier>> enumLayeringSystem =
            new MapLayeringSystem<>(Layers.SAMPLES, pasteurizedGamepadMap);

    LayeredGamepad<EnhancedDoubleSupplier, EnhancedBooleanSupplier, PasteurizedGamepad<EnhancedDoubleSupplier, EnhancedBooleanSupplier>>
            layeredGamepad = new LayeredGamepad<>(enumLayeringSystem);

    private final FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private int servoDownCounter = -1;
    private int clawCounter = -1;
    private int hangCounter = -1;
    private int servoSpecCounter = -1;

    AllMech robot;
    ServoProgramming servo;
    EnhancedBooleanSupplier hangingButton;
    MecanumDrive drive;

    @Override
    public void init() {
        robot = new AllMech(hardwareMap);
        servo = new ServoProgramming(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    @Override
    public void loop() {
        hangingButton = Pasteurized.gamepad1().leftBumper();

        telemetry.addData("Current Layer:", enumLayeringSystem.getLayer());
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();

        if (layeredGamepad.start().state()) {
            enumLayeringSystem.setLayer(Layers.SPECIMEN);
        } else if (layeredGamepad.options().state()) {
            enumLayeringSystem.setLayer(Layers.SAMPLES);
        }

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        .484038 * Math.tan(1.12 * gamepad1.left_stick_y),
                        .484038 * Math.tan(1.12 * gamepad1.left_stick_x)
                ),
                -gamepad1.right_stick_x
        ));

        drive.updatePoseEstimate();

        specLogic();
//        if (specimenGamepad.a().onTrue()) {
//            servoSpecCounter++;
//        }
//        if (servoSpecCounter > 1) {
//            servoSpecCounter = 0;
//        }
//        if (servoSpecCounter == 0) {
//            runningActions.add(
//                    robot.servoSpecimen()
//            );
//        }
//        if (servoSpecCounter == 1) {
//            runningActions.add(
//                    new SequentialAction (
//                            robot.servoSpecimenScore(),
//                            new SleepAction(0.3),
//                            robot.setLinkageTarget(25),
//                            new SleepAction(0.5),
//                            robot.setElevatorTarget(530)
//                    )
//
//            );
//        }

        sampleLogic();

//        if (sampleGamepad.a().onTrue()) {
//            servoDownCounter++;
//        }
//
//        if (servoDownCounter > 1) {
//            servoDownCounter = 0;
//        }
//
//        if (servoDownCounter == 0) {
//            runningActions.add(
//                    robot.servoDown()
//            );
//        }
//        if (servoDownCounter == 1) {
//            runningActions.add(
//                    robot.servoGet()
//            );
//        }

        //claw stuff
        clawLogic();
//        if (layeredGamepad.rightBumper().onTrue()) {
//            clawCounter++;
//        }
//        if (clawCounter > 1) {
//            clawCounter = 0;
//        }
//        if (clawCounter == 0) {
//            runningActions.add(
//                    robot.clawClose()
//            );
//        }
//        if (clawCounter == 1) {
//            runningActions.add(
//                    robot.clawOpen()
//            );
//        }

//hang stuff
//        if (hangingButton.onTrue()) {
//            hangCounter++;
//        }
//        if (hangCounter > 2) {
//            hangCounter = 0;
//        }
//        if (hangCounter == 0) {
//            runningActions.add(
//                    new SequentialAction(
//                            robot.servoGet(),
//                            new SleepAction(0.3),
//                            robot.setHangingTarget(2700)
//                    )
//            );
//        }
//        if (hangCounter == 1) {
//            runningActions.add(
//                    robot.setHangingTarget(1200)
//            );
//        }
//        if (hangCounter == 2) {
//            runningActions.add(
//                    robot.setHangingTarget(0)
//            );
//        }
        hangLogic();

//        if (layeredGamepad.x().onTrue()){
//            runningActions.add(
//                    new InstantAction(() -> servo.rotate.setPosition(servo.rotate.getPosition() + .1))
//            );
//        }
//        if (layeredGamepad.b().onTrue()){
//            runningActions.add(
//                    new InstantAction(() -> servo.rotate.setPosition(servo.rotate.getPosition() - .1))
//
//            );
//        }

        rotateLogic();

        //elevator up
//        if (layeredGamepad.dpadUp().state()) {
//            runningActions.add(
//                    robot.setElevatorTarget(2500)
//            );
//        }

//        if (layeredGamepad.dpadDown().state() && (AllMech.linkage.getCurrentPosition() < -400)) {
//            runningActions.add(
//                    robot.setElevatorTarget(0)
//            );
//        } else if (layeredGamepad.dpadDown().state()) {
//            runningActions.add(
//                    new SequentialAction(
//                            robot.servoDown(),
//                            new SleepAction(0.5),
//                            robot.setElevatorTarget(0)
//                    )
//            );
//        }
        elevatorLogic();

//        if (layeredGamepad.dpadLeft().state()) {
//            runningActions.add(
//                    new ParallelAction(
//                            robot.servoDown(),
//                            robot.setLinkageTarget(25)
//                    )
//            );
//        }
//
//        if (layeredGamepad.dpadRight().state()) {
//            runningActions.add(
//                    robot.setLinkageTarget(-550)
//            );
//        }
        linkageLogic();

        if (sampleGamepad.y().state()) {
            runningActions.add(
                    robot.servoUp()
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

    private void hangLogic() {
        if (hangingButton.onTrue()) {
            hangCounter++;
        }
        if (hangCounter > 2) {
            hangCounter = 0;
        }
        if (hangCounter == 0) {
            runningActions.add(
                    new SequentialAction(
                            robot.servoGet(),
                            new SleepAction(0.3),
                            robot.setHangingTarget(2700)
                    )
            );
        }
        if (hangCounter == 1) {
            runningActions.add(
                    robot.setHangingTarget(1200)
            );
        }
        if (hangCounter == 2) {
            runningActions.add(
                    robot.setHangingTarget(0)
            );
        }
    }

    private void clawLogic() {
        if (layeredGamepad.rightBumper().onTrue()) {
            clawCounter++;
        }
        if (clawCounter > 1) {
            clawCounter = 0;
        }
        if (clawCounter == 0) {
            runningActions.add(
                    robot.clawClose()
            );
        }
        if (clawCounter == 1) {
            runningActions.add(
                    robot.clawOpen()
            );
        }
    }

    private void sampleLogic() {
        if (sampleGamepad.a().onTrue()) {
            servoDownCounter++;
        }

        if (servoDownCounter > 1) {
            servoDownCounter = 0;
        }

        if (servoDownCounter == 0) {
            runningActions.add(
                    robot.servoDown()
            );
        }
        if (servoDownCounter == 1) {
            runningActions.add(
                    robot.servoGet()
            );
        }
    }

    private void specLogic() {
        if (specimenGamepad.a().onTrue()) {
            servoSpecCounter++;
        }
        if (servoSpecCounter > 1) {
            servoSpecCounter = 0;
        }
        if (servoSpecCounter == 0) {
            runningActions.add(
                    robot.servoSpecimen()
            );
        }
        if (servoSpecCounter == 1) {
            runningActions.add(
                    new SequentialAction (
                            robot.servoSpecimenScore(),
                            new SleepAction(0.3),
                            robot.setLinkageTarget(25),
                            new SleepAction(0.5),
                            robot.setElevatorTarget(530)
                    )

            );
        }

        if (gamepad2.dpad_up) {
            runningActions.add(
                    robot.setElevatorTarget(1100)
            );
        }
    }

    private void rotateLogic() {
        if (layeredGamepad.x().onTrue()){
            runningActions.add(
                    new InstantAction(() -> servo.rotate.setPosition(servo.rotate.getPosition() + .1))
            );
        }
        if (layeredGamepad.b().onTrue()){
            runningActions.add(
                    new InstantAction(() -> servo.rotate.setPosition(servo.rotate.getPosition() - .1))

            );
        }
    }

    private void elevatorLogic() {
        if (layeredGamepad.dpadUp().state()) {
            runningActions.add(
                    robot.setElevatorTarget(2500)
            );
        }

        if (layeredGamepad.dpadDown().state() && (AllMech.linkage.getCurrentPosition() < -400)) {
            runningActions.add(
                    robot.setElevatorTarget(0)
            );
        } else if (layeredGamepad.dpadDown().state()) {
            runningActions.add(
                    new SequentialAction(
                            robot.servoDown(),
                            new SleepAction(0.5),
                            robot.setElevatorTarget(0)
                    )
            );
        }
    }

    private void linkageLogic() {
        if (layeredGamepad.dpadLeft().state()) {
            runningActions.add(
                    new ParallelAction(
                            robot.servoDown(),
                            robot.setLinkageTarget(25)
                    )
            );
        }

        if (layeredGamepad.dpadRight().state()) {
            runningActions.add(
                    robot.setLinkageTarget(-550)
            );
        }
    }
}
