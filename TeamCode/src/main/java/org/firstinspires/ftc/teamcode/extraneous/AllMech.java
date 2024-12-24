package org.firstinspires.ftc.teamcode.extraneous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AllMech extends LinearOpMode {

    public DcMotor frontLeft, frontRight, rearRight, rearLeft;
    public static DcMotor linkage, elevator;

    PIDController linkageController;

    public static double pv = 0.0, iv = 0.0, dv = 0.0;
    public static double pl = 0.006, il = 0.0, dl = 0.0009;
    public static double fv = 0.0, fl = 0.07;

    public volatile int linkTarget = 0;

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

        //Drive motor inits
        frontLeft = hardwareMap.get(DcMotor.class, "left front motor");
        frontRight = hardwareMap.get(DcMotor.class, "right front motor");
        rearRight = hardwareMap.get(DcMotor.class, "right rear motor");
        rearLeft = hardwareMap.get(DcMotor.class, "left rear motor");

        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public Action setLinkageTarget(int target) {
        return new InstantAction(() -> linkTarget = target);
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



    @Override
    public void runOpMode() throws InterruptedException {

    }
}
