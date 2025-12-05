package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Simplified Indexer:
 *  - advanceOneSlot(): manual/strong single advance (immediate, uses indexerPower)
 *  - startPreAdvanceOneSlot(): gentle pre-advance with settle (recommended for Auto pattern matching)
 *  - startIntakeAdvanceOneSlot(): gentle advance while intaking (recommended for Auto intake)
 *  - startAutoLaunchAllThree(): stepped (Auto)
 *  - startAutoLaunchAllThreeContinuous(): continuous (TeleOp)
 *
 * UPDATED:
 *  - Active position hold whenever not moving (RUN_TO_POSITION + holdPower)
 *  - Separate settleDelayS after any move finishes inside launch sequences
 *  - Gentle pre-advance state machine (slower power + settle)
 *  - NEW: gentle intake-advance state machine (defaults match pre-advance)
 */
public class Indexer {
    private DcMotorEx indexer;
    private Servo camServo;

    private final String indexerMotorName;
    private final String camServoName;

    private final int ticksPerRev;
    private final int slots;
    private final double indexerPower;   // strong/normal move power (launch advances, manual advance)
    private final double camInitPos;     // cam closed
    private final double camOpenPos;     // cam open
    private boolean homeCamOnInit = true;

    private int ticksPerSlot;
    private int targetPosition = 0;

    private boolean moving = false;
    private boolean camOpen = false;

    private boolean autoRunning = false;
    private int autoStep = 0;
    private double timerS = 0.0;

    // false -> stepped (Auto), true -> continuous (TeleOp)
    private boolean continuousAuto = false;

    // Timing:
    // stepDelayS   = intentional wait BEFORE a feed/move in sequences
    // settleDelayS = wait AFTER a move finishes (in sequences) to reduce bounce/jams
    private double stepDelayS = 0.15;
    private double settleDelayS = 0.10;

    // Active hold power used when idle/not moving
    private double holdPower = 0.08;

    // ===== Gentle pre-advance (pattern matching) settings =====
    private double preAdvancePower = 0.48;          // tune 0.35–0.70
    private double preAdvanceSettleDelayS = 0.18;   // tune 0.08–0.25

    // ===== NEW: Gentle intake-advance settings (defaults match pre-advance) =====
    private double intakeAdvancePower = preAdvancePower;
    private double intakeAdvanceSettleDelayS = preAdvanceSettleDelayS;

    // ===== Gentle-move state machine (shared) =====
    private enum GentleMode { NONE, PREADVANCE, INTAKE }
    private GentleMode gentleMode = GentleMode.NONE;
    private boolean gentleAdvancing = false;
    private int gentleStep = 0; // 0=wait for move finish, 1=settle
    private double gentleSettleDelayS = 0.18;

    public Indexer(String indexerMotorName, String camServoName) {
        this(indexerMotorName, camServoName, 1425, 3, 0.8, 0.65, 0.0);
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

    /** Delay used in BOTH auto stepped and teleop continuous sequences (pre-move wait). */
    public void setStepDelay(double seconds) {
        this.stepDelayS = Math.max(0.0, seconds);
    }

    /** Post-move settle delay used inside launch sequences. */
    public void setSettleDelay(double seconds) {
        this.settleDelayS = Math.max(0.0, seconds);
    }

    /** Idle "hold" power (RUN_TO_POSITION) used when not moving. */
    public void setHoldPower(double power) {
        this.holdPower = Math.max(0.0, Math.min(0.5, power));
        if (indexer != null && !moving) applyHold();
    }

    /** Gentle pre-advance tuning (pattern matching). */
    public void setPreAdvancePower(double power) {
        this.preAdvancePower = Math.max(0.0, Math.min(1.0, power));
        // keep intake defaults matched unless user overrides intake explicitly
        this.intakeAdvancePower = this.preAdvancePower;
    }

    public void setPreAdvanceSettleDelay(double seconds) {
        this.preAdvanceSettleDelayS = Math.max(0.0, seconds);
        this.intakeAdvanceSettleDelayS = this.preAdvanceSettleDelayS;
    }

    /** NEW: optionally tune intake-advance separately (if needed). */
    public void setIntakeAdvancePower(double power) {
        this.intakeAdvancePower = Math.max(0.0, Math.min(1.0, power));
    }

    public void setIntakeAdvanceSettleDelay(double seconds) {
        this.intakeAdvanceSettleDelayS = Math.max(0.0, seconds);
    }

    public void init(HardwareMap hw) {
        indexer = hw.get(DcMotorEx.class, indexerMotorName);
        camServo = hw.get(Servo.class, camServoName);

        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ticksPerSlot = ticksPerRev / Math.max(1, slots);
        targetPosition = indexer.getCurrentPosition();

        moving = false;
        camOpen = false;

        autoRunning = false;
        continuousAuto = false;
        autoStep = 0;
        timerS = 0.0;

        gentleMode = GentleMode.NONE;
        gentleAdvancing = false;
        gentleStep = 0;

        if (homeCamOnInit) camServo.setPosition(camInitPos);

        applyHold();
    }

    public void hardZero() {
        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetPosition = 0;

        moving = false;

        autoRunning = false;
        continuousAuto = false;
        autoStep = 0;
        timerS = 0.0;

        gentleMode = GentleMode.NONE;
        gentleAdvancing = false;
        gentleStep = 0;

        applyHold();
    }

    /** Manual single advance (immediate/strong). */
    public void advanceOneSlot() {
        if (moving || autoRunning || gentleAdvancing) return;
        targetPosition += ticksPerSlot;
        runToTarget(indexerPower);
    }

    /** Gentle pre-advance 1 slot with settle (pattern matching). */
    public void startPreAdvanceOneSlot() {
        startGentleOneSlot(GentleMode.PREADVANCE, preAdvancePower, preAdvanceSettleDelayS);
    }

    /** NEW: Gentle advance 1 slot with settle (use this while intaking). */
    public void startIntakeAdvanceOneSlot() {
        startGentleOneSlot(GentleMode.INTAKE, intakeAdvancePower, intakeAdvanceSettleDelayS);
    }

    public boolean isPreAdvancing() {
        return gentleAdvancing && gentleMode == GentleMode.PREADVANCE;
    }

    public boolean isIntakeAdvancing() {
        return gentleAdvancing && gentleMode == GentleMode.INTAKE;
    }

    private void startGentleOneSlot(GentleMode mode, double power, double settleS) {
        if (moving || autoRunning || gentleAdvancing) return;

        gentleMode = mode;
        gentleAdvancing = true;
        gentleStep = 0;
        timerS = 0.0;
        gentleSettleDelayS = Math.max(0.0, settleS);

        targetPosition += ticksPerSlot;
        runToTarget(power);
    }

    /** Stepped 3-ball launch (Auto). */
    public void startAutoLaunchAllThree() {
        if (autoRunning || moving || gentleAdvancing) return;
        autoRunning = true;
        continuousAuto = false;
        autoStep = 0;
        timerS = 0.0;
        setCamOpen(true);
    }

    /** Continuous 3-ball launch (TeleOp). */
    public void startAutoLaunchAllThreeContinuous() {
        if (autoRunning || moving || gentleAdvancing) return;
        autoRunning = true;
        continuousAuto = true;
        autoStep = 0;
        timerS = 0.0;
        setCamOpen(true);
    }

    public void update(double dt) {
        // Motion end handling
        if (moving && !indexer.isBusy()) {
            moving = false;
            applyHold();
        }

        // Advance timers if any state machine is active
        if (autoRunning || gentleAdvancing) timerS += dt;

        // ===== Gentle one-slot state machine (pre-advance OR intake-advance) =====
        if (gentleAdvancing) {
            switch (gentleStep) {
                case 0:
                    if (!moving) {
                        timerS = 0.0;
                        gentleStep = 1;
                    }
                    break;

                case 1:
                    if (timerS > gentleSettleDelayS) {
                        gentleAdvancing = false;
                        gentleMode = GentleMode.NONE;
                        gentleStep = 0;
                        applyHold();
                    }
                    break;
            }
            return; // don’t run launch state machine same tick
        }

        if (!autoRunning) return;

        if (continuousAuto) {
            switch (autoStep) {
                case 0:
                    if (timerS > stepDelayS) {
                        timerS = 0.0;
                        autoStep = 1;
                        targetPosition += 2 * ticksPerSlot;
                        runToTarget(indexerPower);
                    }
                    break;

                case 1:
                    if (!moving && timerS > settleDelayS) {
                        timerS = 0.0;
                        autoStep = 2;
                    }
                    break;

                case 2:
                    if (timerS > stepDelayS) {
                        setCamOpen(false);
                        autoRunning = false;
                        continuousAuto = false;
                        applyHold();
                    }
                    break;
            }
        } else {
            switch (autoStep) {
                case 0:
                    if (timerS > stepDelayS) {
                        timerS = 0.0;
                        autoStep = 1;
                        targetPosition += ticksPerSlot;
                        runToTarget(indexerPower);
                    }
                    break;

                case 1:
                    if (!moving && timerS > settleDelayS) {
                        timerS = 0.0;
                        autoStep = 2;
                    }
                    break;

                case 2:
                    if (timerS > stepDelayS) {
                        timerS = 0.0;
                        autoStep = 3;
                        targetPosition += ticksPerSlot;
                        runToTarget(indexerPower);
                    }
                    break;

                case 3:
                    if (!moving && timerS > settleDelayS) {
                        timerS = 0.0;
                        autoStep = 4;
                    }
                    break;

                case 4:
                    if (timerS > stepDelayS) {
                        setCamOpen(false);
                        autoRunning = false;
                        applyHold();
                    }
                    break;
            }
        }
    }

    private void runToTarget(double power) {
        indexer.setTargetPosition(targetPosition);
        indexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexer.setPower(power);
        moving = true;
    }

    private void applyHold() {
        if (indexer == null) return;

        if (holdPower <= 0.0) {
            indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            indexer.setPower(0.0);
            return;
        }

        indexer.setTargetPosition(targetPosition);
        indexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexer.setPower(holdPower);
    }

    // --- Helpers ---
    public boolean isMoving() { return moving; }
    public boolean isAutoRunning() { return autoRunning; }

    public void homeCam() { setCamOpen(false); }

    public void setCamOpen(boolean open) {
        camOpen = open;
        camServo.setPosition(camOpen ? camOpenPos : camInitPos);
    }

    public boolean isCamOpen() { return camOpen; }

    public int getTargetPosition() { return targetPosition; }
    public int getCurrentPosition() { return (indexer != null) ? indexer.getCurrentPosition() : 0; }
}


