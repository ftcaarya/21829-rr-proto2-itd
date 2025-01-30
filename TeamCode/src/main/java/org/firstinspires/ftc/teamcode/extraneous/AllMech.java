package org.firstinspires.ftc.teamcode.extraneous;

import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.ARM_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.ARM_SERVO_SCORE;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.ARM_SERVO_SPEC;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.ARM_SERVO_UP;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.LEFT_ARM_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.LEFT_ARM_SERVO_SCORE;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.LEFT_ARM_SERVO_SPEC;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.LEFT_ARM_SERVO_UP;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.WRIST_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.WRIST_SERVO_SPEC;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.WRIST_SERVO_SPEC_SCORE;
import static org.firstinspires.ftc.teamcode.extraneous.ServoProgramming.WRIST_SERVO_UP;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class AllMech extends LinearOpMode {

    public DcMotor frontLeft, frontRight, rearRight, rearLeft;
    public Servo arm, wrist, rotate, claw, leftArm;
    public static DcMotor linkage, elevator;

    public IMU imu;

    PIDController linkageController;
    PIDController vertController;
    PIDController hangingController;

    ServoProgramming servo;

    public static double pv = 0.0055, iv = 0.0, dv = 0.00065;
    public static double pl = 0.014, il = 0.0, dl = 0.0001;
    public static double fv = 0.175, fl = 0.12;

    public volatile int linkTarget = 0;
    public static int vertTarget = 0;
    public static int hangTarget = 0;

    private final double hang_ticks_in_degeres = 537.7/180; // current 312 rpm, 117 rpm is 1,425.1/180
    private final double ticks_in_degree = 384.5/180;


    public AllMech(HardwareMap hardwareMap) {
        linkage = hardwareMap.get(DcMotor.class, "linkage");
        linkage.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator = hardwareMap.get(DcMotor.class, "elevator");


        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linkage.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        System.out.println("subsystem encoders reset");

        linkageController = new PIDController(pl, il, dl);
        vertController = new PIDController(pv, iv, dv);

        servo = new ServoProgramming(hardwareMap);

        arm = hardwareMap.get(Servo.class, "arm servo");
        leftArm = hardwareMap.get(Servo.class, "left arm servo");
        wrist = hardwareMap.get(Servo.class, "wrist servo");
        rotate = hardwareMap.get(Servo.class, "rotate servo");
        claw = hardwareMap.get(Servo.class, "claw servo");

        //Drive motor inits
        frontLeft = hardwareMap.get(DcMotorEx.class, "left front motor");
        frontRight = hardwareMap.get(DcMotorEx.class, "right front motor");
        rearRight = hardwareMap.get(DcMotorEx.class, "right rear motor");
        rearLeft = hardwareMap.get(DcMotorEx.class, "left rear motor");

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        imu.initialize(parameters);

        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);


    }

    public Action servoSpecimen() {
        return new ParallelAction(
                new InstantAction(() -> servo.arm.setPosition(ARM_SERVO_SPEC)),
                new InstantAction(() -> servo.leftArm.setPosition(LEFT_ARM_SERVO_SPEC)),
                new InstantAction(() -> servo.wrist.setPosition(WRIST_SERVO_SPEC))
        );
    }
    public Action moveRotate(double position) {
        return new InstantAction(() -> servo.rotate.setPosition(position));
    }

    public Action servoDown() {

        return new ParallelAction(
                new InstantAction(() -> servo.arm.setPosition(ARM_SERVO_DOWN)),
                new InstantAction(() -> servo.leftArm.setPosition(LEFT_ARM_SERVO_DOWN)),
                new InstantAction(() -> servo.wrist.setPosition(WRIST_SERVO_DOWN))
        );
    }

    public Action servoGet(){

        return new ParallelAction(
                new InstantAction(() -> servo.arm.setPosition(ARM_SERVO_DOWN+.12)),
                new InstantAction(() -> servo.leftArm.setPosition(LEFT_ARM_SERVO_DOWN-0.12)),

                new InstantAction(() -> servo.wrist.setPosition(WRIST_SERVO_DOWN-0.07))
        );





    }

    public Action servoUp(){
        return new ParallelAction(
                new InstantAction(() -> servo.arm.setPosition(ARM_SERVO_UP)),
                new InstantAction(() -> servo.leftArm.setPosition(LEFT_ARM_SERVO_UP)),
                new InstantAction(() -> servo.wrist.setPosition(WRIST_SERVO_UP))
        );
    }

    public Action servoSpecimenScore(){
        return new ParallelAction(
                new InstantAction(() -> servo.arm.setPosition(ARM_SERVO_SCORE)),
                new InstantAction(() -> servo.leftArm.setPosition(LEFT_ARM_SERVO_SCORE)),
                new InstantAction(() -> servo.wrist.setPosition(WRIST_SERVO_SPEC_SCORE))
        );

    }

    public Action clawClose() {
        return new InstantAction(() -> servo.claw.setPosition(CLAW_CLOSE));
    }

    public Action clawOpen() {
        return new InstantAction(() -> servo.claw.setPosition(CLAW_OPEN));
    }



    public Action setLinkageTarget(int target) {
        return new InstantAction(() -> linkTarget = target);
    }

    public Action scoreSample() {
        return new SequentialAction(
                setElevatorTarget(2500),
                new SleepAction(2),
                servoUp(),
                new SleepAction(0.2),
                clawOpen(),
                new SleepAction(0.2)
        );
    }

    public Action pickUpSample() {
        return new SequentialAction(
                servoGet(),
                new SleepAction(0.2),
                clawClose(),
                servoDown(),
                new SleepAction(0.2),
                setElevatorTarget(0),
                new SleepAction(1),
                setLinkageTarget(100),
                new SleepAction(2)
        );
    }

    public Action resetMechs() {
        return new SequentialAction(
                servoDown(),
                new SleepAction(0.3),
                setElevatorTarget(0),
                new SleepAction(1),
                new InstantAction(() -> linkTarget = -250),
                new SleepAction(0.5),
                new InstantAction(() -> linkTarget = -550)
        );
    }

    public Action setElevatorTarget(int target) {
        return new InstantAction(() -> vertTarget = target);
    }

    public class UpdateLinkPID implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            linkageController.setPID(pl, il ,dl);

            int linkagePos = linkage.getCurrentPosition();

            double linkagePID = linkageController.calculate(linkagePos, linkTarget);

            double linkFF = Math.cos(Math.toRadians(linkTarget / ticks_in_degree)) * fl;

            double linkPower = linkagePID + linkFF;

            linkage.setPower(linkPower);
            System.out.println("link target position: " + linkTarget);
            System.out.println("link current position: " + linkagePos);
            System.out.println("link power" + linkPower);

            return true;
        }
    }
    public Action updateLinkPID(){
        return new UpdateLinkPID();
    }

//    public class UpdateHangPID implements Action {
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            hangingController.setPID(ph, ih, dh);
//
//            int hangingPos = hanging.getCurrentPosition();
//
//            double hangingPID = hangingController.calculate(hangingPos, hangTarget);
//
//            double hangFF = Math.cos(Math.toRadians(linkTarget / hang_ticks_in_degeres)) * fh;
//
//            double hangPower = hangingPID + hangFF;
//
//            hanging.setPower(hangPower);
//
//            telemetry.addData("Hanging current position", hangingPos);
//            telemetry.addData("Hanging target position", hangTarget);
//            return true;
//        }
//    }
//
//    public Action updateHangPID() {
//        return new UpdateHangPID();
//    }
//
//    public Action setHangingTarget(int target) {
//        return new InstantAction(() -> hangTarget = target);
//    }

    public class UpdatePID implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            linkageController.setPID(pl, il ,dl);
            vertController.setPID(pv, iv, dv);

            int linkagePos = linkage.getCurrentPosition();
            int vertPos = elevator.getCurrentPosition();

            double linkagePID = linkageController.calculate(linkagePos, linkTarget);
            double vertPID = vertController.calculate(vertPos, vertTarget);

            double linkFF = Math.cos(Math.toRadians(linkTarget / ticks_in_degree)) * fl;
            double vertFF = Math.cos(Math.toRadians(vertTarget / ticks_in_degree)) * fv;

            double linkPower = linkagePID + linkFF;
            double vertPower = vertPID + vertFF;

            elevator.setPower(vertPower);
            linkage.setPower(linkPower);

            return true;
        }
    }

    public class UpdateVertPID implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            vertController.setPID(pv, iv, dv);

            int vertPos = elevator.getCurrentPosition();

            double vertPID = vertController.calculate(vertPos, vertTarget);

            double vertFF = Math.cos(Math.toRadians(vertTarget / ticks_in_degree)) * fv;

            double vertPower = vertPID + vertFF;

            elevator.setPower(vertPower);

            return true;
        }
    }
    public Action updateVertPID() {
        return new UpdateVertPID();
    }

    public Action updatePID() {
        return new UpdatePID();
    }


//    public class ServoSpecimen implements Action {
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            servo.arm.setPosition(ARM_SERVO_SPEC);
//            servo.leftArm.setPosition(LEFT_ARM_SERVO_SPEC);
//            servo.wrist.setPosition(WRIST_SERVO_SPEC);
//            return false;
//        }
//    }
//
//    public Action servoSpecimen() {
//        return new ServoSpecimen();
//    }

//    public class ServoUp implements Action {
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            servo.arm.setPosition(ARM_SERVO_UP);
//            servo.leftArm.setPosition(LEFT_ARM_SERVO_UP);
//            servo.wrist.setPosition(WRIST_SERVO_UP);
//
//            return false;
//        }
//    }
//
//    public Action servoUp() {
//        return new ServoUp();
//    }

//    public class ServoDown implements Action {
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            servo.arm.setPosition(ARM_SERVO_DOWN);
//            servo.leftArm.setPosition(LEFT_ARM_SERVO_DOWN);
//            servo.wrist.setPosition(WRIST_SERVO_DOWN);
//            return false;
//        }
//    }
//    public Action servoDown() {
//        return new ServoDown();
//    }


    @Override
    public void runOpMode() throws InterruptedException {

    }
}