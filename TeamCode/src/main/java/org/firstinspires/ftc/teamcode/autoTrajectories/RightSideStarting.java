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

        Action dropPreloaded = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(0, -32), Math.toRadians(270))
                .build();

        Action getFirstSample = drive.actionBuilder(new Pose2d(0, -32, Math.toRadians(270)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(33, -34.5, Math.toRadians(42)), Math.toRadians(45))
                .build();

        Action dropFirstSample = drive.actionBuilder(new Pose2d(33, -34.5, Math.toRadians(42)))
                .strafeToLinearHeading(new Vector2d(39, -31), Math.toRadians(-80), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-20, 20))
                .build();

        Action getSecondSample = drive.actionBuilder(new Pose2d(39, -32, Math.toRadians(-80)))
                .turnTo(Math.toRadians(35))
                .build();

        Action dropSecondSample = drive.actionBuilder(new Pose2d(39, -32, Math.toRadians(35)))
                .strafeToLinearHeading(new Vector2d(47, -34), Math.toRadians(270))
                .build();

        Action slowGetSpecimen = drive.actionBuilder(new Pose2d(47, -34, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(47, -45), new TranslationalVelConstraint(20.0))
                .build();

        Action slowGetSpecimenNoVel = drive.actionBuilder(new Pose2d(47, -34, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(47, -44))
                .build();

        Action scoreFirstSpecimen = drive.actionBuilder(new Pose2d(47, -45, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(-9, -32))
                .build();


        Action getSecondSpecimenstrafe = drive.actionBuilder(new Pose2d(-9, -32, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(47, -39),Math.toRadians(270),new TranslationalVelConstraint(100))
                .build();

        Action slowGetSecondSpecNoVel = drive.actionBuilder(new Pose2d(47, -39, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(47, -46))
                .build();

        Action scoreSecondSpecimen = drive.actionBuilder(new Pose2d(47, -45.5, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(-6, -32),new TranslationalVelConstraint(80))
                .build();

        Action scoreThirdSpecimen = drive.actionBuilder(new Pose2d(47, -46, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(5, -32))
                .build();

        Action getThirdSpecimen = drive.actionBuilder(new Pose2d(-6, -30 , Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(47, -46), Math.toRadians(270))
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
                                new ParallelAction(
                                        robot.servoSpecimenScore(),
                                        robot.setElevatorTarget(375)
                                ),
                                new SleepAction(0.25),
                                //score preloaded
                                new ParallelAction(
                                        robot.moveRotate(0),
                                        robot.setLinkageTarget(100),
                                        robot.clawClose()

                                ),
                                new SleepAction(0.2),

                                dropPreloaded,
                                new SleepAction(0.2),
                                robot.setElevatorTarget(1700),
                                new SleepAction(1),
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
                                        getFirstSample,
                                        robot.setElevatorTarget(250),
                                        robot.moveRotate(.15)
                                ),
                                robot.servoGet(),
                                new SleepAction(.15),
                                robot.clawClose(),
                                new SleepAction(.15),
                                robot.servoDown(),
                                dropFirstSample,
                                robot.setElevatorTarget(800),
                                new SleepAction(1),


                                //drop first sample
                                robot.clawOpen(),
                                new SleepAction(.2),
                                new ParallelAction(
                                        robot.setElevatorTarget(250),
                                        //get second sample
                                        getSecondSample,
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
                                        dropSecondSample

                                ),
                                robot.clawOpen(),

                                slowGetSpecimenNoVel,
                                new SleepAction(.1),
                                robot.clawClose(),
                                new SleepAction(0.3),
                                robot.setElevatorTarget(0),

                                //score first specimen
                                new ParallelAction(
                                        robot.servoUp(),
                                        robot.setLinkageTarget(100)
                                ),
                                new SleepAction(0.5),


                                new ParallelAction(
                                        robot.setElevatorTarget(375),
                                        robot.servoSpecimenScore()
                                ),
                                scoreFirstSpecimen,
                                new SleepAction(0.2),
                                robot.setElevatorTarget(1700),
                                new SleepAction(1),
                                robot.clawOpen(),
                                new ParallelAction(
                                        robot.servoDown(),
                                        robot.setElevatorTarget(0)
                                ),

                                //get second spec
                                new ParallelAction(
                                        robot.servoSpecimen(),
                                        getSecondSpecimenstrafe,
                                        new SequentialAction(
                                                robot.setLinkageTarget(-250),
                                                new SleepAction(0.5),
                                                robot.setLinkageTarget(-550)
                                        )

                                ),


                                //get second specimen
                                slowGetSecondSpecNoVel,
                                new SleepAction(0.15),
                                robot.clawClose(),
                                new SleepAction(0.1),
                                robot.servoUp(),
                                new SleepAction(0.1),


                                robot.setLinkageTarget(100),
                                new SleepAction(0.5),
                                //score second specimen

                                new ParallelAction(
                                        robot.setElevatorTarget(375),
                                        robot.servoSpecimenScore()
                                ),
                                scoreSecondSpecimen,
                                new SleepAction(0.2),
                                robot.setElevatorTarget(1700),
                                new SleepAction(1),
                                robot.clawOpen(),


                                // reset from scoring
                                new ParallelAction(
                                        robot.servoDown(),
                                        robot.setElevatorTarget(0)
                                ),
                                new SleepAction(0.3),
                                new ParallelAction(
                                        robot.servoSpecimen(),
                                        getThirdSpecimen,
                                        new SequentialAction(
                                                robot.setLinkageTarget(-250),
                                                new SleepAction(0.3),
                                                robot.setLinkageTarget(-550)
                                        )

                                )
                                //get third specimen
//                                new SleepAction(0.1),
//                                robot.clawClose(),
//                                new SleepAction(0.2),
//                                robot.servoUp(),
//                                new SleepAction(0.1),
//
//
//                                //score third specimen
//                                robot.setLinkageTarget(100),
//                                new SleepAction(0.3),
//                                new ParallelAction(
//                                        robot.servoSpecimenScore(),
//                                        scoreThirdSpecimen.build()
//
//                                ),
//                                robot.setElevatorTarget(1700),
//                                new SleepAction(.7),
//                                robot.clawOpen()
                        )
                )


        );


    }
}
