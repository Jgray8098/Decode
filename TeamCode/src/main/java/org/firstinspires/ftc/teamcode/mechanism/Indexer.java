package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Simplified Indexer:
 *  - advanceOneSlot() for manual use
 *  - autoLaunchAllThree(): open cam, advance twice, close cam, done (stepped)
 *  - autoLaunchAllThreeContinuous(): open cam, move 2 slots in one motion, close cam (TeleOp only)
 */
public class Indexer {
    private DcMotorEx indexer;
    private Servo camServo;

    private final String indexerMotorName;
    private final String camServoName;

    private final int ticksPerRev;
    private final int slots;
    private final double indexerPower;
    private final double camInitPos;
    private final double camOpenPos;
    private boolean homeCamOnInit = true;

    private int ticksPerSlot;
    private int targetPosition = 0;
    private boolean moving = false;
    private boolean camOpen = false;
    private boolean autoRunning = false;
    private int autoStep = 0;
    private double timerS = 0.0;

    // --- auto mode flags ---
    // false  -> original stepped 3-shot sequence (Auto)
    // true   -> continuous 2-slot move (TeleOp fast mode)
    private boolean continuousAuto = false;

    // timing (seconds)
    // Instance-level so TeleOp can shorten it without affecting Auto.
    private double stepDelayS = 0.15;
    private static final int POSITION_TOL = 10;

    public Indexer(String indexerMotorName, String camServoName) {
        this(indexerMotorName, camServoName, 1425, 3, 0.9, 0.65, 0.0);
    }

    public Indexer(String indexerMotorName, String camServoName,
                   int ticksPerRev, int slots, double indexerPower,
                   double camInitPos, double camOpenPos) {
        this.indexerMotorName = indexerMotorName;
        this.camServoName = camServoName;
        this.ticksPerRev = ticksPerRev;
        this.slots = slots;
        this.indexerPower = indexerPower;
        this.camInitPos = camInitPos;
        this.camOpenPos = camOpenPos;
    }

    public void setHomeCamOnInit(boolean enable) {
        this.homeCamOnInit = enable;
    }

    /** Allow TeleOp to use a smaller delay between advances without changing Auto behavior. */
    public void setStepDelay(double seconds) {
        // clamp to something reasonable so no one accidentally sets 0
        this.stepDelayS = Math.max(0.05, seconds);
    }

    public void init(HardwareMap hw) {
        indexer = hw.get(DcMotorEx.class, indexerMotorName);
        camServo = hw.get(Servo.class, camServoName);

        //indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ticksPerSlot = ticksPerRev / Math.max(1, slots);
        targetPosition = indexer.getTargetPosition();
        moving = false;
        camOpen = false;
        autoRunning = false;
        continuousAuto = false;
        autoStep = 0;
        timerS = 0.0;

        if (homeCamOnInit) {
            camServo.setPosition(camInitPos);
        }
    }

    public void hardZero() {
        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetPosition = 0;
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** Manual single advance. */
    public void advanceOneSlot() {
        if (moving || autoRunning) return;
        targetPosition += ticksPerSlot;
        runToTarget();
    }

    /**
     * ORIGINAL stepped 3-ball launch (Auto uses this):
     *  - cam open
     *  - wait, advance 1 slot
     *  - wait, advance 1 slot
     *  - wait, cam close
     */
    public void startAutoLaunchAllThree() {
        if (autoRunning || moving) return;
        autoRunning = true;
        continuousAuto = false;   // use stepped mode
        autoStep = 0;
        timerS = 0;
        setCamOpen(true); // launch first ball
    }

    /**
     * NEW continuous 3-ball launch for TeleOp:
     *  - cam open
     *  - single move of 2 slots
     *  - small delay, cam close
     */
    public void startAutoLaunchAllThreeContinuous() {
        if (autoRunning || moving) return;
        autoRunning = true;
        continuousAuto = true;    // use continuous mode
        autoStep = 0;
        timerS = 0;
        setCamOpen(true);         // fire first ball

        // Plan a single continuous move of 2 slots (slot 1 -> slot 3).
        targetPosition += 2 * ticksPerSlot;
        runToTarget();
    }

    public void update(double dt) {
        // Manage motion end (for both manual + auto)
        if (moving && !indexer.isBusy()) {
            indexer.setPower(0.0);
            indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            moving = false;
        }

        if (!autoRunning) return;

        timerS += dt;

        if (continuousAuto) {
            // === CONTINUOUS TELEOP MODE ===
            // Wait until we're done moving 2 slots, then a tiny delay, then close cam.
            if (!moving && timerS > stepDelayS) {
                setCamOpen(false);
                autoRunning = false;
                continuousAuto = false;
            }
        } else {
            // === ORIGINAL STEPPED AUTO MODE ===
            switch (autoStep) {
                case 0:
                    // wait a short delay before first advance
                    if (timerS > stepDelayS) {
                        timerS = 0;
                        autoStep = 1;
                        targetPosition += ticksPerSlot;
                        runToTarget();
                    }
                    break;

                case 1:
                    // wait for first move to finish, then short delay
                    if (!moving && timerS > stepDelayS) {
                        timerS = 0;
                        autoStep = 2;
                        targetPosition += ticksPerSlot;
                        runToTarget();
                    }
                    break;

                case 2:
                    // after second move and delay, close cam and finish
                    if (!moving && timerS > stepDelayS) {
                        setCamOpen(false);
                        autoRunning = false;
                    }
                    break;
            }
        }
    }

    private void runToTarget() {
        indexer.setTargetPosition(targetPosition);
        indexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexer.setPower(indexerPower);
        moving = true;
    }

    // --- Helpers ---
    public boolean isMoving() { return moving; }
    public boolean isAutoRunning() { return autoRunning; }

    public void homeCam() {
        setCamOpen(false); // this will send servo to camInitPos
    }

    public void setCamOpen(boolean open) {
        camOpen = open;
        camServo.setPosition(camOpen ? camOpenPos : camInitPos);
    }

    public boolean isCamOpen() { return camOpen; }
}

