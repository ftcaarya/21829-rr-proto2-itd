package org.firstinspires.ftc.teamcode.teleop;

import static org.slf4j.MDC.put;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier;
import dev.frozenmilk.dairy.core.util.supplier.numeric.EnhancedDoubleSupplier;
import dev.frozenmilk.dairy.pasteurized.PasteurizedGamepad;
import dev.frozenmilk.dairy.pasteurized.SDKGamepad;
import dev.frozenmilk.dairy.pasteurized.layering.LayeredGamepad;
import dev.frozenmilk.dairy.pasteurized.layering.MapLayeringSystem;

public class LayeringTest  extends OpMode {
    enum Layers {
        SAMPLES,
        SPECIMEN
    }

    private final FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    SDKGamepad sampleGamepad;
    SDKGamepad specimenGamepad;
    @Override
    public void init() {
        sampleGamepad = new SDKGamepad(gamepad2);
        specimenGamepad = new SDKGamepad(gamepad2);


    }

    @Override
    public void loop() {
        Map<Layers, PasteurizedGamepad<EnhancedDoubleSupplier, EnhancedBooleanSupplier>> pasteurizedGamepadMap = new HashMap<Layers, PasteurizedGamepad<EnhancedDoubleSupplier, EnhancedBooleanSupplier>>(){{
            put(Layers.SAMPLES, sampleGamepad);
            put(Layers.SPECIMEN, specimenGamepad);
        }};
        MapLayeringSystem<Layers, EnhancedDoubleSupplier, EnhancedBooleanSupplier, PasteurizedGamepad<EnhancedDoubleSupplier, EnhancedBooleanSupplier>> enumLayeringSystem = new MapLayeringSystem<>(Layers.SAMPLES, pasteurizedGamepadMap);

        LayeredGamepad<EnhancedDoubleSupplier, EnhancedBooleanSupplier, PasteurizedGamepad<EnhancedDoubleSupplier, EnhancedBooleanSupplier>> layeredGamepad = new LayeredGamepad<>(enumLayeringSystem);


        telemetry.addData("Current Layer:", enumLayeringSystem.getLayer());
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();

        if (gamepad2.start) {
            enumLayeringSystem.setLayer(Layers.SPECIMEN);
        } else if (gamepad2.options) {
            enumLayeringSystem.setLayer(Layers.SAMPLES);
        }

        if (enumLayeringSystem.isActive(sampleGamepad)) {

        } else if (enumLayeringSystem.isActive(specimenGamepad)) {

        }


        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);

    }
}
