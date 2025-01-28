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

import dev.frozenmilk.dairy.core.util.controller.calculation.pid.DoubleComponent;
import dev.frozenmilk.dairy.core.util.controller.calculation.pid.UnitComponent;

@Autonomous(name = "Left Side Starting Trajectory", group = "exercises")
public class LeftSideStarting extends LinearOpMode {

    AllMech robot;
    ServoProgramming servo;

    private static DcMotor linkage, elevator;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-35, -61, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        robot = new AllMech(hardwareMap);
        servo = new ServoProgramming(hardwareMap);

        TrajectoryActionBuilder dropPreloaded = drive.actionBuilder(initialPose)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-52, -52 , Math.toRadians(45)), -Math.PI);

        TrajectoryActionBuilder getSecondSample = drive.actionBuilder(new Pose2d(-52,-52, -Math.PI))
                .strafeToLinearHeading(new Vector2d(-48, -52), Math.toRadians(90));


        TrajectoryActionBuilder scoreSecondSample = drive.actionBuilder(new Pose2d(-48,-52, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(45));

        TrajectoryActionBuilder getThirdSample = drive.actionBuilder(new Pose2d(-52,-52, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-58, -52), Math.toRadians(90));

        TrajectoryActionBuilder scoreThirdSample = drive.actionBuilder(new Pose2d(-58,-52, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(45));

        TrajectoryActionBuilder getFourthSample = drive.actionBuilder(new Pose2d(-52,-52, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-54, -44), (Math.PI - Math.atan((18/14.5))));

        TrajectoryActionBuilder scoreFourthSample = drive.actionBuilder(new Pose2d(-54,-44, (Math.PI - Math.atan((18/14.5)))))
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(45));

        TrajectoryActionBuilder levelOneAscent = drive.actionBuilder(new Pose2d(-52, -52, Math.toRadians(45)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-24, -8, Math.toRadians(180)), Math.PI/10)
                .stopAndAdd(robot.servoUp());

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
                                        robot.setLinkageTarget(100),
                                        robot.clawClose()
                                ),

                                //position to drop preloaded
                                dropPreloaded.build(),

                                //drop preloaded
                                robot.scoreSample(),

                                //reset mechs
                                robot.resetMechs(),

                                //position to pick up second sample
                                new ParallelAction(
                                        getSecondSample.build(),
                                        robot.setElevatorTarget(550)
                                ),

                                //pick up second sample
                                robot.pickUpSample(),

                                //position to score second sample
                                scoreSecondSample.build(),

                                //score second sample
                                robot.scoreSample(),

                                //reset mechs
                                robot.resetMechs(),

                                //position to pick up third sample
                                new ParallelAction(
                                        getThirdSample.build(),
                                        robot.setElevatorTarget(550)
                                ),

                                //pick up third sample
                                robot.pickUpSample(),

                                //position to score third sample
                                scoreThirdSample.build(),

                                //score third sample
                                robot.scoreSample(),

                                //reset mechs
                                robot.resetMechs(),

                                //position to pick up fourth sample
                                new ParallelAction(
                                        getFourthSample.build(),
                                        robot.setElevatorTarget(350),
                                        robot.moveRotate(0.5)
                                ),

                                // pick up fourth sample
                                robot.pickUpSample(),

                                //position to score fourth sample
                                scoreFourthSample.build(),

                                //score fourth sample
                                robot.scoreSample(),

                                //reset mechs
                                robot.resetMechs(),

                                //achieve level one ascent
                                levelOneAscent.build()
                        )
                )


        );


    }
}
