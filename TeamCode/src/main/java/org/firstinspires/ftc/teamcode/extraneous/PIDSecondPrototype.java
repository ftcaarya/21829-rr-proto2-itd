package org.firstinspires.ftc.teamcode.extraneous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "Testing second prototype PID", group = "exercise")
public class PIDSecondPrototype extends OpMode {
    public PIDController vertController;
    public PIDController linkageController;
    public PIDController hangLeftController;
    public PIDController hangRightController;

    public static double pv = 0.03, iv = 0.0, dv = 0.00065;
    public static double pl = 0.014, il = 0.0, dl = 0.0001;
    public static double ph = 0.0, ih = 0.0, dh = 0.0;
    public static double fv = 0.2, fl = 0.12, fh = 0.0;

    public static int vertTarget;
    public static int hangTarget;
    public static int linkTarget;

    private final double ticks_in_degrees = 384.5/180;
    private final double hang_ticks_in_degrees = 0/180;

    public DcMotor linkage, elevator, hangingLeft, hangingRight;

    @Override
    public void init() {
        vertController = new PIDController(pv, iv, dv);
        linkageController = new PIDController(pl, il, dl);
        hangLeftController = new PIDController(ph, ih, dh);
        hangRightController = new PIDController(ph, ih, dh);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        linkage = hardwareMap.get(DcMotor.class, "linkage");
        linkage.setDirection(DcMotorSimple.Direction.REVERSE);
        
        elevator = hardwareMap.get(DcMotor.class, "elevator");

        hangingLeft = hardwareMap.get(DcMotor.class, "hanging left");
        hangingRight = hardwareMap.get(DcMotor.class, "hanging right");

        hangingRight.setDirection(DcMotorSimple.Direction.REVERSE);
        hangingLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        linkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangingLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangingRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangingLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangingRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linkage.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        vertController.setPID(pv, iv, dv);
        linkageController.setPID(pl, il, dl);
        hangLeftController.setPID(ph, ih, dh);
        hangRightController.setPID(ph, ih, dh);

        int vertPos = elevator.getCurrentPosition();
        int linkagePos = linkage.getCurrentPosition();
        int hangLeftPos = hangingLeft.getCurrentPosition();
        int hangRightPos = hangingRight.getCurrentPosition();

        double vertPID = vertController.calculate(vertPos, vertTarget);
        double linkagePID = linkageController.calculate(linkagePos, linkTarget);
        double hangLeftPID = hangLeftController.calculate(hangLeftPos, hangTarget);
        double hangRightPID = hangRightController.calculate(hangRightPos, hangTarget);

        double vertFF = Math.cos(Math.toRadians(vertTarget / ticks_in_degrees)) * fv;
        double linkFF = Math.cos(Math.toRadians(linkTarget / ticks_in_degrees)) * fl;
        double hangFF = Math.cos(Math.toRadians(hangTarget / hang_ticks_in_degrees)) * fh;

        double vertPower = vertPID + vertFF;
        double linkPower = linkagePID + linkFF;
        double hangRightPower = hangRightPID + hangFF;
        double hangLeftPower = hangLeftPID + hangFF;

        hangingRight.setPower(hangRightPower);
        hangingLeft.setPower(hangLeftPower);
        elevator.setPower(vertPower);
        linkage.setPower(linkPower);

        telemetry.addData("Elevator currnet position", vertPos);
        telemetry.addData("Elevator target position", vertTarget);

        telemetry.addData("Linkage current position", linkagePos);
        telemetry.addData("Linkage target position", linkTarget);

        telemetry.addData("left hang current position", hangLeftPos);
        telemetry.addData("left hang target position", hangTarget);

        telemetry.addData("right hang current position", hangRightPos);
        telemetry.addData("right hang target position", hangTarget);

        telemetry.update();

    }
}
