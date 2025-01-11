package org.firstinspires.ftc.teamcode.autoTrajectories;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.extraneous.AllMech;
import org.firstinspires.ftc.teamcode.extraneous.ServoProgramming;

@Autonomous(name = "Right Side Starting Trajectory", group = "exercises")
public class RightSideStarting extends LinearOpMode {

    AllMech robot;
    ServoProgramming servo;

    private static DcMotor linkage, elevator;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, -61, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        robot = new AllMech(hardwareMap);
        servo = new ServoProgramming(hardwareMap);

        TrajectoryActionBuilder dropPreloaded = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(0, -29), Math.toRadians(270));

        TrajectoryActionBuilder getFirstSample = drive.actionBuilder(new Pose2d(0, -29, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(33, -38), Math.toRadians(45));

        TrajectoryActionBuilder dropFirstSample = drive.actionBuilder(new Pose2d(33, -38, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(38, -40), Math.toRadians(-45));

        TrajectoryActionBuilder getSecondSample = drive.actionBuilder(new Pose2d(38, -40, Math.toRadians(315)))
                .turnTo(35);

        TrajectoryActionBuilder dropSecondSample = drive.actionBuilder(new Pose2d(38, -40, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(47, -47), Math.toRadians(270));

        TrajectoryActionBuilder getFirstSpecimen = drive.actionBuilder(new Pose2d(-47, -47, Math.toRadians(270)))
                .strafeTo(new Vector2d(-47, -50));
        TrajectoryActionBuilder scoreSpecimen = drive.actionBuilder(new Pose2d(47, -50, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(0, -28), Math.toRadians(270));

        TrajectoryActionBuilder scoreSecondSpecimen = drive.actionBuilder(new Pose2d(47, -47.5, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-5, -29), Math.toRadians(270));

        TrajectoryActionBuilder scoreThirdSpecimen = drive.actionBuilder(new Pose2d(47, -47.5, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-9, -29), Math.toRadians(270));


        TrajectoryActionBuilder Move3Spec = drive.actionBuilder(new Pose2d(-9, -28, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(5, -28), new TranslationalVelConstraint(20.0));


        TrajectoryActionBuilder getSpecimen = drive.actionBuilder(new Pose2d(0, -29, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(47, -42), Math.toRadians(270));

        TrajectoryActionBuilder getThirdSpecimen = drive.actionBuilder(new Pose2d(-5, -28, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(47, -42), Math.toRadians(270));

        TrajectoryActionBuilder slowGetSpecimen = drive.actionBuilder(new Pose2d(47, -42, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(47, -47.5), new TranslationalVelConstraint(20.0));


        linkage = hardwareMap.get(DcMotor.class, "linkage");
        elevator = hardwareMap.get(DcMotor.class, "elevator");


        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linkage.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linkage.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Actions.runBlocking(
                new ParallelAction(
                        robot.updatePID(),
                        new SequentialAction(
                                new ParallelAction(
                                        robot.moveRotate(0),
                                        robot.setLinkageTarget(100),
                                        robot.clawClose()
                                ),
                                robot.servoSpecimenScore(),
                                dropPreloaded.build(),
                                new SleepAction(1),
                                robot.setElevatorTarget(1100),
                                new SleepAction(0.8),
                                robot.clawOpen(),

                                robot.setElevatorTarget(-20),
                                robot.setLinkageTarget(-200),
                                new SleepAction(1),
                                robot.setLinkageTarget(-420),
                                robot.servoSpecimen(),
                                getSpecimen.build(),
                                slowGetSpecimen.build(),
                                robot.clawClose(),
                                new SleepAction(0.8),
                                robot.servoUp(),
                                new ParallelAction(
                                        scoreSecondSpecimen.build(),
                                        robot.servoSpecimenScore(),
                                        robot.setLinkageTarget(100)
                                ),
                                new SleepAction(0.5),
                                robot.setElevatorTarget(1100),
                                new SleepAction(0.8),
                                robot.clawOpen(),

                                robot.setElevatorTarget(-20),
                                robot.setLinkageTarget(-200),
                                new SleepAction(1),
                                robot.setLinkageTarget(-420),
                                robot.servoSpecimen(),
                                getThirdSpecimen.build(),
                                slowGetSpecimen.build(),
                                robot.clawClose(),
                                new SleepAction(0.8),
                                robot.servoUp(),
                                new ParallelAction(
                                        scoreThirdSpecimen.build(),
                                        robot.servoSpecimenScore(),
                                        robot.setLinkageTarget(100)
                                ),
                                new SleepAction(0.5),
                                robot.setElevatorTarget(1050),
                                new SleepAction(0.25),
                                Move3Spec.build(),
                                robot.clawOpen()




//                        getSecondSample.build(),
//                        robot.servoGet(),
//                        robot.clawClose(),
//                        dropSecondSample.build(),
//                        robot.setElevatorTarget(-20),
//                        robot.clawOpen(),
//                        robot.servoSpecimen(),
//
//                        getFirstSpecimen.build(),
//                        robot.clawClose(),
//                        robot.servoUp(),
//                        robot.setLinkageTarget(550),
//                        robot.servoSpecimenScore(),
//                        scoreSpecimen.build(),
//                        robot.setElevatorTarget(1100),
//                        new SleepAction(0.25),
//                        robot.clawOpen(),
//                        robot.servoSpecimen(),
//                        robot.setLinkageTarget(0),
//
//                        getSpecimen.build(),
//                        robot.clawClose(),
//                        robot.servoUp(),
//                        robot.setLinkageTarget(550),
//                        robot.servoSpecimenScore(),
//                        scoreSpecimen.build(),
//                        robot.setElevatorTarget(1100),
//                        new SleepAction(0.25),
//                        robot.clawOpen(),
//                        robot.servoSpecimen(),
//                        robot.setLinkageTarget(0),
//
//                        getSpecimen.build(),
//                        robot.clawClose(),
//                        robot.servoUp(),
//                        robot.setLinkageTarget(550),
//                        robot.servoSpecimenScore(),
//                        scoreSpecimen.build(),
//                        robot.setElevatorTarget(1100),
//                        new SleepAction(0.25),
//                        robot.clawOpen(),
//                        robot.servoSpecimen(),
//                        robot.setLinkageTarget(0),
//
//                        getSpecimen.build(),
//                        robot.clawClose(),
//                        robot.servoUp(),
//                        robot.setLinkageTarget(550),
//                        robot.servoSpecimenScore(),
//                        scoreSpecimen.build(),
//                        robot.setElevatorTarget(1100),
//                        new SleepAction(0.25),
//                        robot.clawOpen(),
//                        robot.servoSpecimen(),
//                        robot.setLinkageTarget(0)
                        )
                )


        );


    }
}
