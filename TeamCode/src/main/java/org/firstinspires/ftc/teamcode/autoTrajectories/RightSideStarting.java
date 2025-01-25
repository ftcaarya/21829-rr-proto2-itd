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
                .strafeToLinearHeading(new Vector2d(0, -30.8), Math.toRadians(270));

        TrajectoryActionBuilder getFirstSample = drive.actionBuilder(new Pose2d(0, -30.8, Math.toRadians(270)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(34, -35.5, Math.toRadians(42)), Math.toRadians(45));

        TrajectoryActionBuilder dropFirstSample = drive.actionBuilder(new Pose2d(34, -35.5, Math.toRadians(42)))
                .strafeToLinearHeading(new Vector2d(40, -32), Math.toRadians(-80), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-20, 20));

        TrajectoryActionBuilder getSecondSample = drive.actionBuilder(new Pose2d(40, -32, Math.toRadians(-80)))
                .turnTo(Math.toRadians(35));

        TrajectoryActionBuilder dropSecondSample = drive.actionBuilder(new Pose2d(40, -32, Math.toRadians(35)))
                .strafeToLinearHeading(new Vector2d(47, -34), Math.toRadians(270));

        TrajectoryActionBuilder slowGetSpecimen = drive.actionBuilder(new Pose2d(47, -34, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(47, -45), new TranslationalVelConstraint(20.0));

        TrajectoryActionBuilder slowGetSpecimenNoVel = drive.actionBuilder(new Pose2d(47, -34, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(47, -45));

        TrajectoryActionBuilder scoreFirstSpecimen = drive.actionBuilder(new Pose2d(47, -45, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-7, -30.7), Math.toRadians(270),new TranslationalVelConstraint(100));


        TrajectoryActionBuilder getSecondSpecimenstrafe = drive.actionBuilder(new Pose2d(-7, -32, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(47, -39),Math.toRadians(270),new TranslationalVelConstraint(100));

        TrajectoryActionBuilder slowGetSecondSpecNoVel = drive.actionBuilder(new Pose2d(47, -39, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(47, -47));

        TrajectoryActionBuilder scoreSecondSpecimen = drive.actionBuilder(new Pose2d(47, -47, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-9, -29.4), Math.toRadians(270),new TranslationalVelConstraint(100));

        TrajectoryActionBuilder scoreThirdSpecimen = drive.actionBuilder(new Pose2d(47, -46, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(5, -29.7), Math.toRadians(270));

        TrajectoryActionBuilder getThirdSpecimen = drive.actionBuilder(new Pose2d(5, -30.7, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(47, -46), Math.toRadians(270));





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
                                robot.setElevatorTarget(1700),
                                new SleepAction(.6),
                                robot.clawOpen(),

                                //pick first sample
                                new ParallelAction(
                                        robot.servoDown(),
                                        robot.setElevatorTarget(0)

                                ),
                                robot.setLinkageTarget(-200),

                                new SleepAction(0.5),
                                new ParallelAction(
                                        robot.setLinkageTarget(-550),
                                        getFirstSample.build(),
                                        robot.setElevatorTarget(250),
                                        robot.moveRotate(.15)
                                ),
                                robot.servoGet(),
                                new SleepAction(.15),
                                robot.clawClose(),
                                new SleepAction(.15),
                                robot.servoDown(),
                                dropFirstSample.build(),
                                robot.setElevatorTarget(1700),
                                new SleepAction(0.2),


                                //drop first sample
                                robot.clawOpen(),
                                new SleepAction(.2),
                                new ParallelAction(
                                        robot.setElevatorTarget(250),

                                        //get second sample
                                        getSecondSample.build(),
                                        robot.moveRotate(0.22)
                                        ),

                                robot.servoGet(),
                                new SleepAction(.15),
                                robot.clawClose(),
                                new SleepAction(.15),

                                //drop second and get first specimen
                                new ParallelAction(
                                        robot.servoSpecimen(),
                                        robot.moveRotate(0),
                                        dropSecondSample.build()

                                ),
                                robot.clawOpen(),

                                slowGetSpecimenNoVel.build(),
                                new SleepAction(.1),
                                robot.clawClose(),
                                new SleepAction(0.3),

                                //score first specimen
                                robot.servoUp(),
                                new SleepAction(0.1),
                                robot.setLinkageTarget(-300),
                                new SleepAction(0.3),

                                new ParallelAction(
                                        robot.setLinkageTarget(100),
                                        robot.servoSpecimenScore(),
                                        scoreFirstSpecimen.build()

                                ),
                                robot.setElevatorTarget(1700),
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
                                new SleepAction(0.15),
                                robot.clawClose(),
                                new SleepAction(0.1),
                                robot.servoUp(),
                                new SleepAction(0.1),


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
                                robot.setElevatorTarget(1700),
                                new SleepAction(0.6),
                                robot.clawOpen(),


                               // reset from scoring
                                new ParallelAction(
                                        robot.servoDown(),
                                        robot.setElevatorTarget(0)
                                ),
                                new SleepAction(0.3),
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
                                new SleepAction(0.1),
                                robot.clawClose(),
                                new SleepAction(0.2),
                                robot.servoUp(),
                                new SleepAction(0.1),


                                //score third specimen
                                new ParallelAction(
                                        new SequentialAction(
                                                robot.setLinkageTarget(-300),
                                                new SleepAction(0.5),
                                                robot.setLinkageTarget(100)

                                        ),
                                        robot.servoSpecimenScore(),
                                        scoreThirdSpecimen.build()

                                ),
                                robot.setElevatorTarget(1700),
                                new SleepAction(.8),
                                robot.clawOpen()



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
