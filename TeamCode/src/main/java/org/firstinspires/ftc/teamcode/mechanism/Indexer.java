// ===============================
// Indexer.java (FULLY UPDATED - forward-only pocket sync, no init motion, + JOG MODE)
// ===============================
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
 *  - Gentle intake-advance state machine
 *
 * Forward-only pocket re-sync:
 *  - syncToNextPocketForward(true) will move ONLY forward to the next pocket (never reverse)
 *  - or do nothing if already within tolerance of a pocket.
 *
 * INIT behavior:
 *  - cam motion controlled by homeCamOnInit (you currently have it TRUE)
 *
 * NEW: JOG MODE (belt slip / jam recovery)
 *  - enterJogMode(): forces cam closed, cancels sequences, motor switches to RUN_WITHOUT_ENCODER
 *  - jog(stickX): applies manual power (both directions) while keeping cam closed
 *  - exitJogMode(resetEncoder): stops motor; optionally resets encoder so current position becomes "0 pocket"
 *  - update(): does nothing while in jogMode (prevents state machines from fighting you)
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

    // NOTE: you currently have this TRUE in your pasted file.
    // This means the cam will move (to closed) during init() for both Auto and TeleOp.
    // If you want "cam moves during init for Auto but NOT TeleOp", keep this TRUE here,
    // and in TeleOp set indexer.setHomeCamOnInit(false) BEFORE init(hardwareMap).
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
    private double stepDelayS = 0.15;
    private double settleDelayS = 0.10;

    // Active hold power used when idle/not moving
    private double holdPower = 0.08;

    // ===== Gentle pre-advance (pattern matching) settings =====
    private double preAdvancePower = 0.48;          // tune 0.35–0.70
    private double preAdvanceSettleDelayS = 0.18;   // tune 0.08–0.25

    // ===== Gentle intake-advance settings =====
    private double intakeAdvancePower = preAdvancePower;
    private double intakeAdvanceSettleDelayS = preAdvanceSettleDelayS;

    // ===== Gentle-move state machine (shared) =====
    private enum GentleMode { NONE, PREADVANCE, INTAKE }
    private GentleMode gentleMode = GentleMode.NONE;
    private boolean gentleAdvancing = false;
    private int gentleStep = 0; // 0=wait for move finish, 1=settle
    private double gentleSettleDelayS = 0.18;

    // ===== Forward-only pocket sync settings =====
    private static final int POCKET_TOL_TICKS = 20; // tune 10–40 if needed

    // ===== NEW: Jog / Service mode =====
    private boolean jogMode = false;
    private double jogDeadband = 0.10;  // stick deadband
    private double jogMaxPower = 0.35;  // cap jog power (tune 0.20–0.50)

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

    /** If true, we will set cam to "closed" during init(). */
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
        this.holdPower = clamp(power, 0.0, 0.5);
        if (indexer != null && !moving && !jogMode) applyHold();
    }

    /** Gentle pre-advance tuning (pattern matching). */
    public void setPreAdvancePower(double power) {
        this.preAdvancePower = clamp(power, 0.0, 1.0);
        this.intakeAdvancePower = this.preAdvancePower;
    }

    public void setPreAdvanceSettleDelay(double seconds) {
        this.preAdvanceSettleDelayS = Math.max(0.0, seconds);
        this.intakeAdvanceSettleDelayS = this.preAdvanceSettleDelayS;
    }

    public void setIntakeAdvancePower(double power) {
        this.intakeAdvancePower = clamp(power, 0.0, 1.0);
    }

    public void setIntakeAdvanceSettleDelay(double seconds) {
        this.intakeAdvanceSettleDelayS = Math.max(0.0, seconds);
    }

    // ===== NEW: Jog tuning =====
    public boolean isJogMode() { return jogMode; }
    public void setJogMaxPower(double p) { jogMaxPower = clamp(p, 0.05, 1.0); }
    public void setJogDeadband(double d) { jogDeadband = clamp(d, 0.0, 0.5); }

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

        jogMode = false;

        // Cam init behavior (your choice per OpMode via setHomeCamOnInit)
        if (homeCamOnInit && camServo != null) camServo.setPosition(camInitPos);

        // Hold where we are (no movement commanded if already there)
        applyHold();
    }

    public void hardZero() {
        if (indexer == null) return;

        // Exit jog if somehow active
        jogMode = false;
        indexer.setPower(0.0);

        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        if (jogMode) return;
        if (moving || autoRunning || gentleAdvancing) return;
        targetPosition += ticksPerSlot;
        runToTarget(indexerPower);
    }

    public void startPreAdvanceOneSlot() {
        if (jogMode) return;
        startGentleOneSlot(GentleMode.PREADVANCE, preAdvancePower, preAdvanceSettleDelayS);
    }

    public void startIntakeAdvanceOneSlot() {
        if (jogMode) return;
        startGentleOneSlot(GentleMode.INTAKE, intakeAdvancePower, intakeAdvanceSettleDelayS);
    }

    public boolean isPreAdvancing() {
        return gentleAdvancing && gentleMode == GentleMode.PREADVANCE;
    }

    public boolean isIntakeAdvancing() {
        return gentleAdvancing && gentleMode == GentleMode.INTAKE;
    }

    private void startGentleOneSlot(GentleMode mode, double power, double settleS) {
        if (jogMode) return;
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
        if (jogMode) return;
        if (autoRunning || moving || gentleAdvancing) return;
        autoRunning = true;
        continuousAuto = false;
        autoStep = 0;
        timerS = 0.0;
        setCamOpen(true);
    }

    /** Continuous 3-ball launch (TeleOp). */
    public void startAutoLaunchAllThreeContinuous() {
        if (jogMode) return;
        if (autoRunning || moving || gentleAdvancing) return;
        autoRunning = true;
        continuousAuto = true;
        autoStep = 0;
        timerS = 0.0;
        setCamOpen(true);
    }

    public void syncToNextPocketForward(boolean moveToPocket) {
        if (jogMode) return;
        syncToNextPocketForward(moveToPocket, preAdvancePower);
    }

    public void syncToNextPocketForward(boolean moveToPocket, double power) {
        if (jogMode) return;
        if (indexer == null) return;

        // Kill any state machines
        autoRunning = false;
        continuousAuto = false;
        autoStep = 0;

        gentleMode = GentleMode.NONE;
        gentleAdvancing = false;
        gentleStep = 0;

        timerS = 0.0;

        // Stop motor and put in a known mode
        indexer.setPower(0.0);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moving = false;

        double slot = ticksPerRev / (double) Math.max(1, slots);
        int curr = indexer.getCurrentPosition();

        int nearest = (int) Math.round(Math.round(curr / slot) * slot);

        int target;
        if (Math.abs(curr - nearest) <= POCKET_TOL_TICKS) {
            target = nearest;
        } else {
            long k = (long) Math.floor(curr / slot);
            target = (int) Math.round((k + 1) * slot);
            if (target < curr) {
                target = (int) Math.round((k + 2) * slot);
            }
        }

        targetPosition = target;

        if (moveToPocket) {
            runToTarget(clamp(power, 0.0, 1.0));
        } else {
            applyHold();
        }
    }

    // =========================================================
    // NEW: Jog Mode API
    // =========================================================

    /** Enter jog mode: force cam closed, cancel sequences, and allow manual power control. */
    public void enterJogMode() {
        if (indexer == null) return;

        // Cancel everything
        autoRunning = false;
        continuousAuto = false;
        autoStep = 0;

        gentleMode = GentleMode.NONE;
        gentleAdvancing = false;
        gentleStep = 0;

        moving = false;
        timerS = 0.0;

        // Protect cam
        setCamOpen(false);

        // Direct power mode for jogging
        indexer.setPower(0.0);
        indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        jogMode = true;
    }

    /**
     * Apply jog power while in jog mode.
     * stickX: [-1..1] from gamepad2.left_stick_x
     */
    public void jog(double stickX) {
        if (!jogMode || indexer == null) return;

        // Always keep cam protected during jog
        setCamOpen(false);

        double x = stickX;
        if (Math.abs(x) < jogDeadband) x = 0.0;

        // optional: slightly soften near center for fine positioning
        // x = x * x * Math.signum(x);

        double pwr = clamp(x * jogMaxPower, -jogMaxPower, jogMaxPower);
        indexer.setPower(pwr);
    }

    /**
     * Exit jog mode.
     * If resetEncoder=true: current physical pocket becomes "zero" reference.
     */
    public void exitJogMode(boolean resetEncoder) {
        if (indexer == null) return;

        // Stop and protect cam
        indexer.setPower(0.0);
        setCamOpen(false);

        if (resetEncoder) {
            indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            targetPosition = 0;
        } else {
            indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            targetPosition = indexer.getCurrentPosition();
        }

        jogMode = false;

        // Resume normal hold behavior
        applyHold();
    }

    // =========================================================

    public void update(double dt) {

        // If in jog mode, do NOT run state machines / RUN_TO_POSITION logic.
        if (jogMode) {
            // Keep cam protected
            setCamOpen(false);
            return;
        }

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
            return;
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
        if (indexer == null) return;
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
        if (camServo != null) {
            camServo.setPosition(camOpen ? camOpenPos : camInitPos);
        }
    }

    public boolean isCamOpen() { return camOpen; }

    public int getTargetPosition() { return targetPosition; }
    public int getCurrentPosition() { return (indexer != null) ? indexer.getCurrentPosition() : 0; }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}




