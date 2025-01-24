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

@Autonomous(name = "Right Side push", group = "exercises")
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
        TrajectoryActionBuilder slowGetSpecimenNoVel = drive.actionBuilder(new Pose2d(47, -34, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(47, -43.5));
        TrajectoryActionBuilder getSecondSpecimenstrafe = drive.actionBuilder(new Pose2d(-7, -32, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(47, -39),Math.toRadians(270),new TranslationalVelConstraint(100));
        TrajectoryActionBuilder slowGetSecondSpecNoVel = drive.actionBuilder(new Pose2d(47, -39, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(47, -48));
        TrajectoryActionBuilder PosFirstSample = drive.actionBuilder(new Pose2d(33, -35, Math.toRadians(270)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(45,-7,Math.toRadians(270)),Math.toRadians(45));

        TrajectoryActionBuilder pushFirstSample = drive.actionBuilder(new Pose2d(45, -7, Math.toRadians(270)))
                .strafeTo(new Vector2d(45,-52));
        TrajectoryActionBuilder PosSecondSample = drive.actionBuilder(new Pose2d(45, -52, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(53,-10,Math.toRadians(270)),Math.toRadians(45));
        TrajectoryActionBuilder pushSecondSample = drive.actionBuilder(new Pose2d(53, -10, Math.toRadians(270)))
                .strafeTo(new Vector2d(53,-52));

        TrajectoryActionBuilder gotToSpec = drive.actionBuilder(new Pose2d(53, -52, Math.toRadians(270)))
                .strafeTo(new Vector2d(47,-34));










        TrajectoryActionBuilder slowGetSpecimen = drive.actionBuilder(new Pose2d(47, -34, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(47, -45), new TranslationalVelConstraint(20.0));

        TrajectoryActionBuilder scoreFirstSpecimen = drive.actionBuilder(new Pose2d(47, -45, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-7, -32), Math.toRadians(270));

        TrajectoryActionBuilder getSecondSpecimen = drive.actionBuilder(new Pose2d(-7, -32, Math.toRadians(270)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(47, -39, Math.toRadians(270)), Math.PI/9);

        TrajectoryActionBuilder slowGetSecondSpec = drive.actionBuilder(new Pose2d(47, -39, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(47, -48), new TranslationalVelConstraint(15.0));
        TrajectoryActionBuilder slowGetThirdSpec = drive.actionBuilder(new Pose2d(47, -39, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(47, -46));

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
                                robot.setElevatorTarget(0),
                                //score preloaded
                                new ParallelAction(
                                        robot.moveRotate(0),
                                        robot.setLinkageTarget(100),
                                        robot.clawClose(),
                                        robot.servoSpecimenScore()

                                ),

                                dropPreloaded.build(),
                                robot.setElevatorTarget(1350),
                                new SleepAction(.6),
                                robot.clawOpen(),
                                new ParallelAction(
                                        robot.setElevatorTarget(0),
                                        PosFirstSample.build()

                                ),
                                pushFirstSample.build(),
                                PosSecondSample.build(),
                                pushSecondSample.build(),

                                slowGetSpecimenNoVel.build(),
                                new SleepAction(.1),
                                robot.clawClose(),
                                new SleepAction(0.3),

                                //score first specimen
                                robot.servoUp(),
                                robot.setLinkageTarget(-300),
                                new SleepAction(0.3),

                                new ParallelAction(
                                        robot.setLinkageTarget(100),
                                        robot.servoSpecimenScore(),
                                        scoreFirstSpecimen.build()

                                ),
                                robot.setElevatorTarget(1350),
                                new SleepAction(.6),
                                robot.clawOpen(),
                                new ParallelAction(
                                        robot.servoDown(),
                                        robot.setElevatorTarget(0)
                                ),

                                //get second spec
                                new ParallelAction(
                                        robot.servoSpecimen(),
                                        getSecondSpecimenstrafe.build(),
                                        new SequentialAction(
                                                robot.setLinkageTarget(-250),
                                                new SleepAction(0.5),
                                                robot.setLinkageTarget(-550)
                                        )

                                ),


                                //get second specimen
                                slowGetSecondSpecNoVel.build(),
                                new SleepAction(0.1),
                                robot.clawClose(),
                                new SleepAction(0.3),
                                robot.servoUp(),


                                //score second specimen
                                new ParallelAction(
                                        new SequentialAction(
                                                robot.setLinkageTarget(-300),
                                                new SleepAction(0.3),
                                                robot.setLinkageTarget(100)

                                        ),
                                        robot.servoSpecimenScore(),
                                        scoreSecondSpecimen.build()

                                ),
                                robot.setElevatorTarget(1350),
                                robot.clawOpen(),


                                // reset from scoring
                                new ParallelAction(
                                        robot.servoDown(),
                                        robot.setElevatorTarget(0)
                                ),
                                new ParallelAction(
                                        robot.servoSpecimen(),
                                        getThirdSpecimen.build(),
                                        new SequentialAction(
                                                robot.setLinkageTarget(-250),
                                                new SleepAction(0.3),
                                                robot.setLinkageTarget(-550)
                                        )

                                ),


                                //get third specimen
                                slowGetThirdSpec.build(),
                                new SleepAction(0.1),
                                robot.clawClose(),
                                new SleepAction(0.3),
                                robot.servoUp(),


                                //score third specimen
                                new ParallelAction(
                                        new SequentialAction(
                                                robot.setLinkageTarget(-300),
                                                new SleepAction(0.3),
                                                robot.setLinkageTarget(100)

                                        ),
                                        robot.servoSpecimenScore(),
                                        scoreThirdSpecimen.build()

                                ),
                                robot.setElevatorTarget(1350),
                                new SleepAction(.8),
                                robot.clawOpen()

                        )
                )


        );


    }
}
