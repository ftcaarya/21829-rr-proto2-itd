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

    public static double pv = 0.006, iv = 0.0, dv = 0.0007;
    public static double pl = 0.006, il = 0.0, dl = 0.0009;
    public static double fv = 0.175, fl = 0.07;

    public static int vertTarget;
    public static int linkTarget;

    private final double ticks_in_degrees = 576.7/180;

    public DcMotor linkage, elevator;

    @Override
    public void init() {
        vertController = new PIDController(pv, iv, dv);
        linkageController = new PIDController(pl, il, dl);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        linkage = hardwareMap.get(DcMotor.class, "linkage");
        linkage.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator = hardwareMap.get(DcMotor.class, "elevator");

        linkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linkage.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        vertController.setPID(pv, iv, dv);
        linkageController.setPID(pl, il, dl);

        int vertPos = elevator.getCurrentPosition();
        int linkagePos = linkage.getCurrentPosition();

        double vertPID = vertController.calculate(vertPos, vertTarget);
        double linkagePID = linkageController.calculate(linkagePos, linkTarget);

        double vertFF = Math.cos(Math.toRadians(vertTarget / ticks_in_degrees)) * fv;
        double linkFF = Math.cos(Math.toRadians(linkTarget / ticks_in_degrees)) * fl;

        double vertPower = vertPID + vertFF;
        double linkPower = linkagePID + linkFF;

        elevator.setPower(vertPower);
        linkage.setPower(linkPower);

        telemetry.addData("Elevator currnet position", vertPos);
        telemetry.addData("Elevator target position", vertTarget);
        telemetry.addData("Linkage current position", linkagePos);
        telemetry.addData("Linkage target position", linkTarget);

        telemetry.update();

    }
}
