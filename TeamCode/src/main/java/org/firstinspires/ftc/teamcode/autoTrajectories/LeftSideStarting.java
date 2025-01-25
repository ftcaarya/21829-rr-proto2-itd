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
        Pose2d initialPose = new Pose2d(-10, -61, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        robot = new AllMech(hardwareMap);
        servo = new ServoProgramming(hardwareMap);

        TrajectoryActionBuilder dropPreloaded = drive.actionBuilder(initialPose)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-54,-54,Math.toRadians(45)), -Math.toRadians(180));

        TrajectoryActionBuilder getFirstSample = drive.actionBuilder(new Pose2d(-54,-54,Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-48,-40),Math.toRadians(90));

        TrajectoryActionBuilder ScoreFirstSample = drive.actionBuilder(new Pose2d(-48,-40,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(45));

        TrajectoryActionBuilder getSecondSample = drive.actionBuilder(new Pose2d(-54,-54,Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-58,-40),Math.toRadians(90));

        TrajectoryActionBuilder ScoreSecondSample = drive.actionBuilder(new Pose2d(-58,-40,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(45));

        TrajectoryActionBuilder getThirdSample = drive.actionBuilder(new Pose2d(-54,-54,Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-58,-40),Math.toRadians(135));

        TrajectoryActionBuilder ScoreThirdSample = drive.actionBuilder(new Pose2d(-58,-40,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(45));

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
                                robot.setElevatorTarget(0),
                                robot.setLinkageTarget(100),
                                new ParallelAction(
                                        robot.clawClose(),
                                        dropPreloaded.build(),
                                        robot.servoUp()
                                ),
                                robot.setElevatorTarget(2500),
                                new SleepAction(1),
                                robot.clawOpen(),
                                robot.servoDown(),
                                robot.setElevatorTarget(0),
                                new SleepAction(1),
                                robot.setLinkageTarget(-200),
                                new SleepAction(0.5),
                                new ParallelAction(
                                        robot.setLinkageTarget(-550),
                                        getFirstSample.build()

                                ),
                                robot.servoGet(),
                                new SleepAction(0.1),
                                robot.clawClose(),
                                new SleepAction(0.1),
                                robot.servoDown(),
                                new ParallelAction(
                                        ScoreFirstSample.build(),
                                        new SequentialAction(
                                                robot.setLinkageTarget(-300),
                                                new SleepAction(0.5),
                                                robot.setLinkageTarget(100)

                                        )
                                ),
                                robot.setElevatorTarget(2500),
                                new SleepAction(1),
                                robot.servoUp(),
                                new SleepAction(1),
                                robot.clawOpen(),
                                robot.servoDown(),
                                robot.setElevatorTarget(0),
                                new SleepAction(1),
                                robot.setLinkageTarget(-200),
                                new SleepAction(0.5),
                                new ParallelAction(
                                        robot.setLinkageTarget(-550),
                                        getSecondSample.build()

                                ),
                                robot.servoGet(),
                                new SleepAction(0.1),
                                robot.clawClose(),
                                new SleepAction(0.1),
                                robot.servoDown(),
                                new ParallelAction(
                                        ScoreSecondSample.build(),
                                        new SequentialAction(
                                                robot.setLinkageTarget(-300),
                                                new SleepAction(0.5),
                                                robot.setLinkageTarget(100)
                                        )
                                ),
                                robot.setElevatorTarget(2500),
                                new SleepAction(1),
                                robot.servoUp(),
                                new SleepAction(1),
                                robot.clawOpen(),
                                robot.servoDown(),
                                robot.setElevatorTarget(0),
                                new SleepAction(1),
                                robot.setLinkageTarget(-200),
                                new SleepAction(0.5),
                                new ParallelAction(
                                        robot.setLinkageTarget(-550),
                                        getThirdSample.build()

                                ),
                                robot.servoGet(),
                                new SleepAction(0.1),
                                robot.clawClose(),
                                new SleepAction(0.1),
                                robot.servoDown(),
                                new ParallelAction(
                                        ScoreThirdSample.build(),
                                        new SequentialAction(
                                                robot.setLinkageTarget(-300),
                                                new SleepAction(0.5),
                                                robot.setLinkageTarget(100)
                                        )
                                )






                        )
                )


        );


    }
}
