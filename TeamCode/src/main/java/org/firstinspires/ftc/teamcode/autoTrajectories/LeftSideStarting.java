package org.firstinspires.ftc.teamcode.autoTrajectories;

import com.acmerobotics.roadrunner.Action;
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

import dev.frozenmilk.dairy.core.util.controller.calculation.pid.DoubleComponent;
import dev.frozenmilk.dairy.core.util.controller.calculation.pid.UnitComponent;

@Autonomous(name = "Left Side Starting Trajectory", group = "exercises")
public class LeftSideStarting extends LinearOpMode {

    AllMech robot;
    ServoProgramming servo;

    private static DcMotor linkage, elevator;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-33, -61, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        robot = new AllMech(hardwareMap);
        servo = new ServoProgramming(hardwareMap);

        Action dropPreloaded = drive.actionBuilder(initialPose)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-50, -50 , Math.toRadians(45)), -Math.PI)
                .build();

        Action getSecondSample = drive.actionBuilder(new Pose2d(-50,-50, -Math.PI))
                .strafeToLinearHeading(new Vector2d(-47, -47), Math.toRadians(90))
                .build();


        Action scoreSecondSample = drive.actionBuilder(new Pose2d(-47,-47, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.toRadians(45))
                .build();

        Action getThirdSample = drive.actionBuilder(new Pose2d(-50,-50, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-59, -47), Math.toRadians(90))
                .build();

        Action scoreThirdSample = drive.actionBuilder(new Pose2d(-59,-47, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.toRadians(45))
                .build();

        Action getFourthSample = drive.actionBuilder(new Pose2d(-50,-50, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-56, -41), (Math.PI - Math.atan((18/14.5))))
                .build();

        Action scoreFourthSample = drive.actionBuilder(new Pose2d(-54,-41, (Math.PI - Math.atan((18/14.5)))))
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.toRadians(45))
                .build();

        Action levelOneAscent = drive.actionBuilder(new Pose2d(-52, -52, Math.toRadians(45)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-24, 0, Math.toRadians(180)), Math.PI/13, new TranslationalVelConstraint(100), new ProfileAccelConstraint(-100, 100))
                .build();

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
                                //make sure everything is in the right place.
                                new ParallelAction(
                                        robot.setElevatorTarget(0),
                                        robot.setLinkageTarget(-75),
                                        robot.clawClose(),
                                        robot.servoDown(),
                                        robot.moveRotate(0)
                                ),

                                //position to drop preloaded
                                dropPreloaded,

                                //drop preloaded
                                robot.scoreSample(),

                                //reset mechs
                                robot.resetMechs(),

                                //position to pick up second sample
                                new ParallelAction(
                                        getSecondSample,
                                        robot.setElevatorTarget(600)
                                ),
//                                new SleepAction(0.4),

                                //pick up second sample
                                robot.pickUpSample(),

                                //position to score second sample
                                scoreSecondSample,

                                //score second sample
                                robot.scoreSample(),

                                //reset mechs
                                robot.resetMechs(),

                                //position to pick up third sample
                                new ParallelAction(
                                        getThirdSample,
                                        robot.setElevatorTarget(600)
                                ),
//                                new SleepAction(.4),

                                //pick up third sample
                                robot.pickUpSample(),

                                //position to score third sample
                                scoreThirdSample,

                                //score third sample
                                robot.scoreSample(),

                                //reset mechs
                                robot.resetMechs(),
//
                                //position to pick up fourth sample
                                new ParallelAction(
                                        getFourthSample,
                                        robot.setElevatorTarget(490),
                                        robot.moveRotate(0.5)
                                ),

//                                new SleepAction(.4),

                                // pick up fourth sample
                                robot.pickUpSample(),

                                //position to score fourth sample
                                scoreFourthSample,

                                //score fourth sample
                                robot.scoreSample(),

                                //reset mechs
//                                robot.resetMechs(),
                                robot.servoDown(),
                                new SleepAction(0.3),
                                robot.setElevatorTarget(0),
                                new SleepAction(0.25),
//
                                //achieve level one ascent
                                new ParallelAction(
                                        levelOneAscent,
                                        robot.setLinkageTarget(-75)
                                ),
                                robot.servoAscent()

                        )
                )


        );


    }
}
