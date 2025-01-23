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
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.extraneous.AllMech;
import org.firstinspires.ftc.teamcode.extraneous.ServoProgramming;

@Autonomous(name = "Right Side Starting Trajectory", group = "exercises")
public class RightSideStarting1 extends LinearOpMode {

    AllMech robot;
    ServoProgramming servo;

    private static DcMotor linkage, elevator;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10, -61, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        robot = new AllMech(hardwareMap);
        servo = new ServoProgramming(hardwareMap);

        TrajectoryActionBuilder dropPreloaded = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(0, -32.5), Math.toRadians(270));

                ;
        TrajectoryActionBuilder strafeRight = drive.actionBuilder(new Pose2d(0, -32.5, Math.toRadians(270)))
                .strafeTo(new Vector2d(33,-35));


        TrajectoryActionBuilder PosFirstSample = drive.actionBuilder(new Pose2d(33, -35, Math.toRadians(270)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(45,-7,Math.toRadians(270)),Math.toRadians(45));

        TrajectoryActionBuilder pushFirstSample = drive.actionBuilder(new Pose2d(45, -7, Math.toRadians(270)))
                .strafeTo(new Vector2d(45,-52));
        TrajectoryActionBuilder PosSecondSample = drive.actionBuilder(new Pose2d(45, -52, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(53,-10,Math.toRadians(270)),Math.toRadians(45));
        TrajectoryActionBuilder pushSecondSample = drive.actionBuilder(new Pose2d(53, -10, Math.toRadians(270)))
                .strafeTo(new Vector2d(53,-52));







        TrajectoryActionBuilder slowGetSpecimen = drive.actionBuilder(new Pose2d(47, -34, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(47, -45), new TranslationalVelConstraint(20.0));

        TrajectoryActionBuilder scoreFirstSpecimen = drive.actionBuilder(new Pose2d(47, -45, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-7, -32), Math.toRadians(270));

        TrajectoryActionBuilder getSecondSpecimen = drive.actionBuilder(new Pose2d(-7, -32, Math.toRadians(270)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(47, -39, Math.toRadians(270)), Math.PI/9);

        TrajectoryActionBuilder slowGetSecondSpec = drive.actionBuilder(new Pose2d(47, -39, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(47, -48), new TranslationalVelConstraint(15.0));

        TrajectoryActionBuilder scoreSecondSpecimen = drive.actionBuilder(new Pose2d(47, -48, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-9, -31), Math.toRadians(270));

        TrajectoryActionBuilder moveSpecs = drive.actionBuilder(new Pose2d(-9, -31, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(5, -31), new TranslationalVelConstraint(10.0));

        TrajectoryActionBuilder scoreSpecimen = drive.actionBuilder(new Pose2d(47,-50,Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(0,-36), Math.toRadians(270));
        TrajectoryActionBuilder scoreThirdSpecimen = drive.actionBuilder(new Pose2d(47, -47.5, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-9, -29), Math.toRadians(270));


        TrajectoryActionBuilder Move3Spec = drive.actionBuilder(new Pose2d(-9, -28, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(5, -28), new TranslationalVelConstraint(20.0));

        TrajectoryActionBuilder getThirdSpecimen = drive.actionBuilder(new Pose2d(-5, -28, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(47, -42), Math.toRadians(270));





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
                                robot.setElevatorTarget(-20)
                        )
                )
                                //score preloaded
//                                new ParallelAction(
//                                        robot.moveRotate(0),
//                                        robot.setLinkageTarget(100),
//                                        robot.clawClose(),
//                                        robot.servoSpecimenScore()
//
//                                ),
//
//                                dropPreloaded.build(),
//                                robot.setElevatorTarget(1350),
//                                new SleepAction(.6),
//                                robot.clawOpen(),
//
//                                //pick first sample
//                                new ParallelAction(
//                                        robot.servoDown(),
//                                        robot.setElevatorTarget(-20)
//
//                                ),
//                                robot.setLinkageTarget(-200),
//                                new SleepAction(0.3),
//                                new ParallelAction(
//                                        robot.setLinkageTarget(-550),
//                                        getFirstSample.build(),
//                                        robot.setElevatorTarget(250),
//                                        robot.moveRotate(.15)
//                                ),
//
//                                robot.servoGet(),
//                                new SleepAction(.3),
//                                robot.clawClose(),
//                                new SleepAction(.3),
//
//                                new ParallelAction(
//                                        robot.servoDown(),
//                                        robot.setElevatorTarget(900),
//                                        dropFirstSample.build()
//
//                                ),
//
//
//                                //drop first sample
//                                robot.clawOpen(),
//                                new SleepAction(.3),
//                                new ParallelAction(
//                                        robot.setElevatorTarget(250),
//
//                                        //get second sample
//                                        getSecondSample.build(),
//                                        robot.moveRotate(0.17)
//                                ),
//
//                                robot.servoGet(),
//                                new SleepAction(.2),
//                                robot.clawClose(),
//                                new SleepAction(.3),
//
//                                //drop second and get first specimen
//                                new ParallelAction(
//                                        robot.servoSpecimen(),
//                                        robot.moveRotate(0),
//                                        dropSecondSample.build()
//
//                                ),
//                                robot.clawOpen(),
//
//                                slowGetSpecimen.build(),
//                                new SleepAction(.1),
//                                robot.clawClose(),
//                                new SleepAction(0.3),
//
//                                //score first specimen
//                                robot.servoUp(),
//                                robot.setLinkageTarget(-300),
//                                new SleepAction(0.3),
//
//                                new ParallelAction(
//                                        robot.setLinkageTarget(100),
//                                        robot.servoSpecimenScore(),
//                                        scoreFirstSpecimen.build()
//
//                                ),
//                                robot.setElevatorTarget(1350),
//                                new SleepAction(.8),
//                                robot.clawOpen(),
//                                new ParallelAction(
//                                        robot.servoDown(),
//                                        robot.setElevatorTarget(-20)
//                                ),
//                                new ParallelAction(
//                                        robot.servoSpecimen(),
//                                        getSecondSpecimen.build(),
//                                        new SequentialAction(
//                                                robot.setLinkageTarget(-250),
//                                                new SleepAction(0.3),
//                                                robot.setLinkageTarget(-550)
//                                        )
//
//                                ),
//
//
//                                //get second specimen
//
//
//                                new SleepAction(0.4),
//                                slowGetSecondSpec.build(),
//                                new SleepAction(0.5),
//                                robot.clawClose(),
//                                new SleepAction(0.5),
//                                robot.servoUp(),
//
//
//                                //score second specimen
//                                new ParallelAction(
//                                        new SequentialAction(
//                                                robot.setLinkageTarget(-300),
//                                                new SleepAction(0.3),
//                                                robot.setLinkageTarget(100)
//
//                                        ),
//                                        robot.servoSpecimenScore(),
//                                        scoreSecondSpecimen.build()
//
//                                ),
//                                robot.setElevatorTarget(1350),
//                                new SleepAction(.8),
//                                moveSpecs.build(),
//                                robot.clawOpen()

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



        );


    }
}
