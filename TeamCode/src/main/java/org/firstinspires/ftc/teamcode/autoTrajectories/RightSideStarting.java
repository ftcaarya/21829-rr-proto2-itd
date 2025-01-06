package org.firstinspires.ftc.teamcode.autoTrajectories;

import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.CLAW_OPEN;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.extraneous.AllMech;
import org.firstinspires.ftc.teamcode.extraneous.ServoProgramming;

@Autonomous(name = "Right Side Starting Trajectory", group = "exercises")
public class RightSideStarting extends LinearOpMode {

    AllMech robot;
    ServoProgramming servo;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, -61, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        robot = new AllMech(hardwareMap);
        servo = new ServoProgramming(hardwareMap);

        TrajectoryActionBuilder dropPreloaded = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(0,-36));

        TrajectoryActionBuilder getFirstSample = drive.actionBuilder(new Pose2d(0,-36,Math.toRadians(90)))
                        .strafeToLinearHeading(new Vector2d(38,-40),Math.toRadians(45));

        TrajectoryActionBuilder dropFirstSample = drive.actionBuilder(new Pose2d(38,-40,Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(47,-40),Math.toRadians(315));

        TrajectoryActionBuilder getSecondSample = drive.actionBuilder(new Pose2d(47,-40,Math.toRadians(315)))
                .turnTo(35);

        TrajectoryActionBuilder dropSecondSample = drive.actionBuilder(new Pose2d(47,-40,Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(47,-47), Math.toRadians(270));

        TrajectoryActionBuilder getFirstSpecimen = drive.actionBuilder(new Pose2d(-47,-47,Math.toRadians(270)))
                .strafeTo(new Vector2d(-47,-50));
        TrajectoryActionBuilder scoreSpecimen = drive.actionBuilder(new Pose2d(47,-50,Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(0,-36), Math.toRadians(90));

        TrajectoryActionBuilder getSpecimen = drive.actionBuilder(new Pose2d(0,-36,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(47,-50), Math.toRadians(270));








        waitForStart();
        Actions.runBlocking(

                new ParallelAction(
                 robot.updateLinkPID(),
                robot.updateVertPID(),

                new SequentialAction(
                        robot.clawClose,
                        dropPreloaded.build(),
                        robot.setElevatorTarget(1200),
                        robot.setLinkageTarget(550),
                        robot.servoDown,
                        robot.setElevatorTarget(-20),
                        robot.clawOpen,
                        robot.setLinkageTarget(0),
                        getFirstSample.build(),
                        robot.setElevatorTarget(500),
                        robot.clawClose,
                        dropFirstSample.build(),
                        robot.clawOpen,
                        getSecondSample.build(),
                        robot.clawClose,
                        dropSecondSample.build(),
                        robot.clawOpen,
                        robot.servoSpecimen,
                        robot.setElevatorTarget(-20),
                        getFirstSpecimen.build(),
                        robot.clawClose,
                        robot.servoDown,
                        robot.setLinkageTarget(550),
                        robot.setElevatorTarget(1200),
                        scoreSpecimen.build(),
                        robot.setElevatorTarget(-20),
                        robot.clawOpen,

                        getSpecimen.build(),
                        robot.clawClose,
                        robot.servoDown,
                        robot.setLinkageTarget(550),
                        robot.setElevatorTarget(1200),
                        scoreSpecimen.build(),
                        robot.setElevatorTarget(-20),
                        robot.clawOpen,

                        getSpecimen.build(),
                        robot.clawClose,
                        robot.servoDown,
                        robot.setLinkageTarget(550),
                        robot.setElevatorTarget(1200),
                        scoreSpecimen.build(),
                        robot.setElevatorTarget(-20),
                        robot.clawOpen,

                        getSpecimen.build(),
                        robot.clawClose,
                        robot.servoDown,
                        robot.setLinkageTarget(550),
                        robot.setElevatorTarget(1200),
                        scoreSpecimen.build(),
                        robot.setElevatorTarget(-20),
                        robot.clawOpen
                )



                )




        );


    }
}
