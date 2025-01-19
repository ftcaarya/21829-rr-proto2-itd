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
public class RightSideStarting extends LinearOpMode {

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
                .strafeToLinearHeading(new Vector2d(0, -32), Math.toRadians(270));

        TrajectoryActionBuilder getFirstSample = drive.actionBuilder(new Pose2d(0, -32, Math.toRadians(270)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(33, -35, Math.toRadians(40)), Math.toRadians(45));

        TrajectoryActionBuilder dropFirstSample = drive.actionBuilder(new Pose2d(33, -35, Math.toRadians(40)))
                .strafeToLinearHeading(new Vector2d(40, -32), Math.toRadians(-80));

        TrajectoryActionBuilder getSecondSample = drive.actionBuilder(new Pose2d(40, -32, Math.toRadians(-80)))
                .turnTo(Math.toRadians(35));

        TrajectoryActionBuilder dropSecondSample = drive.actionBuilder(new Pose2d(40, -32, Math.toRadians(35)))
                .strafeToLinearHeading(new Vector2d(47, -34), Math.toRadians(270));

        TrajectoryActionBuilder slowGetSpecimen = drive.actionBuilder(new Pose2d(47, -34, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(47, -45), new TranslationalVelConstraint(20.0));

        TrajectoryActionBuilder scoreFirstSpecimen = drive.actionBuilder(new Pose2d(47, -45, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-5, -32), Math.toRadians(270));

        TrajectoryActionBuilder getSecondSpecimen = drive.actionBuilder(new Pose2d(0, -28, Math.toRadians(270)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(47, -42, Math.toRadians(270)), Math.PI/9)
                .strafeToConstantHeading(new Vector2d(47, -47.5), new TranslationalVelConstraint(20.0));

        TrajectoryActionBuilder scoreSecondSpecimen = drive.actionBuilder(new Pose2d(47, -47.5, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-9, -29), Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(5, -28), new TranslationalVelConstraint(20.0));

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
                                robot.setElevatorTarget(-20),
                                //score preloaded
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

                                //pick first sample
                                robot.servoDown(),
                                robot.setElevatorTarget(-20),
                                robot.setLinkageTarget(-200),
                                new SleepAction(1),
                                robot.setLinkageTarget(-550),
                                getFirstSample.build(),
                                robot.setElevatorTarget(250),
                                robot.moveRotate(.15),
                                robot.servoGet(),
                                new SleepAction(.5),
                                robot.clawClose(),
                                new SleepAction(.5),
                                robot.servoDown(),
                                robot.setElevatorTarget(900),


                                //drop first sample
                                dropFirstSample.build(),
                                robot.clawOpen(),
                                new SleepAction(.3),
                                robot.setElevatorTarget(250),


                                getSecondSample.build(),
                                robot.servoGet(),
                                new SleepAction(.2),
                                robot.clawClose(),
                                new SleepAction(.5),

                                robot.servoSpecimen(),
                                dropSecondSample.build(),
                                robot.clawOpen(),
                                robot.moveRotate(0),
                                new SleepAction(3),
                                slowGetSpecimen.build(),
                                new SleepAction(.5),
                                robot.clawClose(),

                                robot.servoUp(),
                                robot.setLinkageTarget(100),
                                robot.servoSpecimenScore(),
                                scoreFirstSpecimen.build()

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
