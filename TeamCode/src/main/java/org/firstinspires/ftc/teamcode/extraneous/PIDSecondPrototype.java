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
    public PIDController hangingController;

    public static double pv = 0.0055, iv = 0.0, dv = 0.00065;
    public static double pl = 0.014, il = 0.0, dl = 0.0001;
    public static double ph = 0.0, ih = 0.0, dh = 0.0;
    public static double fv = 0.175, fl = 0.12, fh = 0.0;

    public static int vertTarget;
    public static int linkTarget;
    public static int hangTarget;

    private final double hang_ticks_in_degeres = 537.7/180; // current 312 rpm, 117 rpm is 1,425.1/180
    private final double ticks_in_degrees = 576.7/180;

    public DcMotor linkage, elevator, hanging;

    @Override
    public void init() {
        vertController = new PIDController(pv, iv, dv);
        linkageController = new PIDController(pl, il, dl);
        hangingController = new PIDController(ph, ih, dh);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        linkage = hardwareMap.get(DcMotor.class, "linkage");
        linkage.setDirection(DcMotorSimple.Direction.REVERSE);
        
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        
        hanging = hardwareMap.get(DcMotor.class, "hanging");
        hanging.setDirection(DcMotorSimple.Direction.FORWARD);

        linkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hanging.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        hanging.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linkage.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        vertController.setPID(pv, iv, dv);
        linkageController.setPID(pl, il, dl);
        hangingController.setPID(ph, ih, dh);
        
        int vertPos = elevator.getCurrentPosition();
        int linkagePos = linkage.getCurrentPosition();
        int hangingPos = hanging.getCurrentPosition();

        double vertPID = vertController.calculate(vertPos, vertTarget);
        double linkagePID = linkageController.calculate(linkagePos, linkTarget);
        double hangingPID = hangingController.calculate(hangingPos, hangTarget);

        double vertFF = Math.cos(Math.toRadians(vertTarget / ticks_in_degrees)) * fv;
        double linkFF = Math.cos(Math.toRadians(linkTarget / ticks_in_degrees)) * fl;
        double hangFF = Math.cos(Math.toRadians(linkTarget / hang_ticks_in_degeres)) * fh;

        double vertPower = vertPID + vertFF;
        double linkPower = linkagePID + linkFF;
        double hangPower = hangingPID + hangFF;

        handing.setPower(hangPower);
        elevator.setPower(vertPower);
        linkage.setPower(linkPower);

        telemetry.addData("Elevator currnet position", vertPos);
        telemetry.addData("Elevator target position", vertTarget);
        telemetry.addData("Linkage current position", linkagePos);
        telemetry.addData("Linkage target position", linkTarget);
        telemetry.addData("Hanging current position", hangingPos);
        telemetry.addData("Hanging target position", hangTarget);

        telemetry.update();

    }
}
