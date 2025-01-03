package org.firstinspires.ftc.teamcode.extraneous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AllMech extends LinearOpMode {

    public DcMotor frontLeft, frontRight, rearRight, rearLeft;
    public Servo arm, wrist, rotate, claw, leftArm;
    public static DcMotor linkage, elevator;

    PIDController linkageController;
    PIDController vertController;

    public static double pv = 0.0055, iv = 0.0, dv = 0.00065;
    public static double pl = 0.014, il = 0.0, dl = 0.001;
    public static double fv = 0.175, fl = 0.15;

    public volatile int linkTarget = 0;
    public static int vertTarget;


    private final double ticks_in_degree = 576.7/180;


    public AllMech(HardwareMap hardwareMap) {
        linkage = hardwareMap.get(DcMotor.class, "linkage");
        linkage.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator = hardwareMap.get(DcMotor.class, "elevator");

        linkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linkage.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linkageController = new PIDController(pl, il, dl);
        vertController = new PIDController(pv, iv, dv);

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


        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);


    }

    public Action setLinkageTarget(int target) {
        return new InstantAction(() -> linkTarget = target);
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





    @Override
    public void runOpMode() throws InterruptedException {

    }
}
