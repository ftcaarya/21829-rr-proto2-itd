package org.firstinspires.ftc.teamcode.extraneous;

import com.qualcomm.robotcore.hardware.Servo;
import java.util.Map;
import java.util.HashMap;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * A standalone class for controlling multiple Servos with acceleration/deceleration
 * and optional synchronized movements so they finish in the same duration.
 */
public class MultiServoController {

    // Servo boundaries
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;

    /**
     * Inner class holding all movement data for a single Servo.
     */
    private static class ServoData {
        Servo  hardwareServo;         // The actual hardware servo object

        double initialPosition;       // Starting position for the move
        double targetPosition;        // Target position
        double currentPosition;       // Current position (what we send to the servo)

        long   moveStartTime;         // System time (ms) when the move started
        double moveDurationSeconds;  // How long this move should take (seconds)

        // Acc/dec shape parameters
        double accelerationRate;      // "Shape" factor for acceleration
        double decelerationRate;      // "Shape" factor for deceleration

        public ServoData(Servo servo) {
            this.hardwareServo       = servo;
            this.initialPosition     = SERVO_MIN;
            this.targetPosition      = SERVO_MIN;
            this.currentPosition     = SERVO_MIN;
            this.moveStartTime       = 0L;
            this.moveDurationSeconds = 1.0; // Default to 1 second
            this.accelerationRate    = 1.0; // Default shape factors
            this.decelerationRate    = 1.0;
        }
    }

    /**
     * Map of servo names -> movement data.
     */
    private final Map<String, ServoData> servoMap = new HashMap<>();

    /**
     * Executor for periodically updating the servo positions in a background thread.
     */
    private ScheduledExecutorService executorService;

    /**
     * How often (in milliseconds) we update all servos.
     */
    private final long updateIntervalMs;

    /**
     * Create a MultiServoController that updates servos on a fixed interval.
     *
     * @param updateIntervalMs period in milliseconds between updates
     *                         (e.g. 20 ms => 50 updates/sec).
     */
    public MultiServoController(long updateIntervalMs) {
        this.updateIntervalMs = updateIntervalMs;
    }

    /**
     * Registers a servo with the given name and hardware object.
     * Uses the default SERVO_MIN as the initial position (0.0).
     *
     * @param name  a unique identifier for this servo
     * @param servo the Servo hardware object from the FTC hardwareMap
     */
    public synchronized void addServo(String name, Servo servo) {
        addServo(name, servo, SERVO_MIN);
    }

    /**
     * Registers a servo with the given name, hardware object,
     * and an explicit initial position (0..1).
     *
     * @param name            a unique identifier for this servo
     * @param servo           the Servo hardware object
     * @param initialPosition the initial position [0..1] at which to set the servo
     */
    public synchronized void addServo(String name, Servo servo, double initialPosition) {
        // Clamp initial position
        double clamped = Math.max(SERVO_MIN, Math.min(SERVO_MAX, initialPosition));

        // Create the ServoData and configure it
        ServoData data = new ServoData(servo);
        data.initialPosition  = clamped;
        data.currentPosition  = clamped;
        data.targetPosition   = clamped;
        data.hardwareServo.setPosition(clamped); // Immediately set hardware

        // Put in the map
        servoMap.put(name, data);
    }

    /**
     * Starts the background update loop if not already running.
     */
    public synchronized void start() {
        if (executorService != null && !executorService.isShutdown()) {
            // Already running
            return;
        }
        executorService = Executors.newSingleThreadScheduledExecutor();
        executorService.scheduleAtFixedRate(
                this::updateAllServos,
                0,
                updateIntervalMs,
                TimeUnit.MILLISECONDS
        );
    }

    /**
     * Stops the background update loop, if running.
     */
    public synchronized void stop() {
        if (executorService != null) {
            executorService.shutdown();
            executorService = null;
        }
    }

    /**
     * Move one servo from its current position to the target over the given duration,
     * shaped by specified acceleration and deceleration rates.
     *
     * @param servoName         the key name of the servo in servoMap
     * @param target            target position [0..1]
     * @param durationSeconds   total move time
     * @param accelerationRate  shape factor for starting (higher => gentler start)
     * @param decelerationRate  shape factor for ending (higher => gentler finish)
     */
    public synchronized void setServoTargetWithAcceleration(
            String servoName,
            double target,
            double durationSeconds,
            double accelerationRate,
            double decelerationRate
    ) {
        ServoData data = servoMap.get(servoName);
        if (data == null) return;

        // Clamp target
        target = Math.max(SERVO_MIN, Math.min(SERVO_MAX, target));

        // Capture current position as the start
        data.initialPosition = data.currentPosition;
        data.targetPosition  = target;

        // Movement timing
        data.moveStartTime       = System.currentTimeMillis();
        data.moveDurationSeconds = durationSeconds;

        // Acc/dec shape
        data.accelerationRate  = accelerationRate;
        data.decelerationRate  = decelerationRate;
    }

    /**
     * Synchronize the movements of two servos so they begin at the same time and
     * both complete in the same duration. Each servo’s distance can differ, but
     * they both finish in 'durationSeconds' with the same shape factors.
     */
    public synchronized void synchronizeServosWithAcceleration(
            String servoName1, double target1,
            String servoName2, double target2,
            double durationSeconds,
            double accelerationRate,
            double decelerationRate
    ) {
        ServoData sd1 = servoMap.get(servoName1);
        ServoData sd2 = servoMap.get(servoName2);
        if (sd1 == null || sd2 == null) return;

        // Clamp targets
        target1 = Math.max(SERVO_MIN, Math.min(SERVO_MAX, target1));
        target2 = Math.max(SERVO_MIN, Math.min(SERVO_MAX, target2));

        // Common start time
        long startTime = System.currentTimeMillis();

        // Setup servo1
        sd1.initialPosition     = sd1.currentPosition;
        sd1.targetPosition      = target1;
        sd1.moveStartTime       = startTime;
        sd1.moveDurationSeconds = durationSeconds;
        sd1.accelerationRate    = accelerationRate;
        sd1.decelerationRate    = decelerationRate;

        // Setup servo2
        sd2.initialPosition     = sd2.currentPosition;
        sd2.targetPosition      = target2;
        sd2.moveStartTime       = startTime;
        sd2.moveDurationSeconds = durationSeconds;
        sd2.accelerationRate    = accelerationRate;
        sd2.decelerationRate    = decelerationRate;
    }

    /**
     * Called periodically (by the executorService) to update all servos.
     * Uses an S-curve function to smoothly accelerate and decelerate.
     */
    private synchronized void updateAllServos() {
        long now = System.currentTimeMillis();

        for (ServoData data : servoMap.values()) {
            double elapsedMs      = now - data.moveStartTime;
            double elapsedSeconds = elapsedMs / 1000.0;

            // If we've reached or exceeded the planned time, snap to target
            if (elapsedSeconds >= data.moveDurationSeconds) {
                data.currentPosition = data.targetPosition;
            } else {
                // 0 <= t < 1
                double t = elapsedSeconds / data.moveDurationSeconds;

                // S-curve shaping
                double shapedT = sCurve(t, data.accelerationRate, data.decelerationRate);

                // Interpolate
                double distance = data.targetPosition - data.initialPosition;
                data.currentPosition = data.initialPosition + distance * shapedT;
            }

            // Clamp just in case
            data.currentPosition = Math.max(SERVO_MIN, Math.min(SERVO_MAX, data.currentPosition));

            // Update hardware servo
            data.hardwareServo.setPosition(data.currentPosition);
        }
    }

    /**
     * An S-curve shaping function that goes from 0 to 1 as t goes from 0 to 1,
     * with controllable "accelerationRate" (a) and "decelerationRate" (d).
     *
     * shape(t) = [ t^(1 + a) ] / [ t^(1 + a) + (1 - t)^(1 + d) ]
     *
     * - shape(0) = 0
     * - shape(1) = 1
     * - Higher 'a' => gentler (slower) start
     * - Higher 'd' => gentler (slower) finish
     */
    private double sCurve(double t, double a, double d) {
        // Handle edge cases
        if (t <= 0.0) return 0.0;
        if (t >= 1.0) return 1.0;

        double up   = Math.pow(t,     1.0 + a);
        double down = Math.pow(1.0 - t, 1.0 + d);
        return up / (up + down);
    }

    public synchronized double getCurrentPosition(String servoName) {
        ServoData data = servoMap.get(servoName);
        if (data != null) {
            return data.currentPosition;
        }
        return -1.0; // Sentinel for "not found"
    }
}

