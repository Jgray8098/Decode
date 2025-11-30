package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * TeleOp-focused Indexer with dedicated PIDF position control.
 *
 * - DOES NOT reset encoder in init(), so encoder position is preserved
 *   when transitioning from Auto (which already calls hardZero()) to TeleOp.
 * - Same public behavior as the Auto Indexer:
 *      * advanceOneSlot()
 *      * startAutoLaunchAllThree()  (3-ball automatic launch)
 * - Uses custom PIDF loop while in "moving" state, with a short "kick" at the
 *   start of each move to overcome static friction and spring load.
 */
public class IndexerTeleOpPIDF {

    private DcMotorEx indexer;
    private Servo camServo;

    private final String indexerMotorName;
    private final String camServoName;

    private final int ticksPerRev;
    private final int slots;
    private final double basePower;   // nominal max power (used as clamp)
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

    // === PIDF coefficients (tune these) ===
    // Start with mostly-P controller – easier to stabilize.
    private double kP = 0.0055; // tweak between ~0.004–0.007 as needed
    private double kI = 0.0;
    private double kD = 0.0;
    private double kF = 0.0;    // not really needed for this use case

    // PID state
    private double integral = 0.0;
    private double lastError = 0.0;

    // --- Move behavior constants ---
    private static final double STEP_DELAY = 0.15;

    // how close we need to be in encoder ticks
    private static final int POSITION_TOL = 10;

    // require being within POSITION_TOL for this long before we stop (seconds)
    private static final double WITHIN_TOL_HOLD_S = 0.05;

    // "kick" to overcome spring & static friction at start of each move
    private static final double KICK_TIME_S  = 0.10;  // duration of kick
    private static final double KICK_POWER   = 0.7;   // high-ish but < 1.0

    // per-move timing
    private boolean kickPhase = false;
    private double moveTimerS = 0.0;
    private double withinTolTimerS = 0.0;

    public IndexerTeleOpPIDF(String indexerMotorName, String camServoName) {
        this(indexerMotorName, camServoName, 1425, 3, 1.0, 0.65, 0.0);
    }

    public IndexerTeleOpPIDF(String indexerMotorName, String camServoName,
                             int ticksPerRev, int slots,
                             double basePower, double camInitPos, double camOpenPos) {
        this.indexerMotorName = indexerMotorName;
        this.camServoName = camServoName;
        this.ticksPerRev = ticksPerRev;
        this.slots = slots;
        this.basePower = basePower;
        this.camInitPos = camInitPos;
        this.camOpenPos = camOpenPos;
    }

    public void setHomeCamOnInit(boolean enable) {
        this.homeCamOnInit = enable;
    }

    /**
     * IMPORTANT: We do NOT reset encoders here.
     * Auto should have already called hardZero() on its own Indexer class.
     * That encoder position carries into TeleOp.
     */
    public void init(HardwareMap hw) {
        indexer = hw.get(DcMotorEx.class, indexerMotorName);
        camServo = hw.get(Servo.class, camServoName);

        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ticksPerSlot = ticksPerRev / Math.max(1, slots);

        // Use current position as starting target so we don't "jump" on enable
        targetPosition = indexer.getCurrentPosition();
        moving = false;
        camOpen = false;

        if (homeCamOnInit) {
            camServo.setPosition(camInitPos);
        }

        // Clear PID state
        integral = 0.0;
        lastError = 0.0;
        kickPhase = false;
        moveTimerS = 0.0;
        withinTolTimerS = 0.0;
    }

    // Optional: expose PID tuning if you want to adjust from a config class or the DS.
    public void setPIDFCoefficients(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /** Manual single advance. */
    public void advanceOneSlot() {
        if (moving || autoRunning) return;
        targetPosition += ticksPerSlot;
        startMoveToTarget();
    }

    /** Begin automatic 3-ball launch (open cam + 2 advances). */
    public void startAutoLaunchAllThree() {
        if (autoRunning || moving) return;
        autoRunning = true;
        autoStep = 0;
        timerS = 0.0;
        setCamOpen(true); // launch first ball immediately
    }

    /**
     * Must be called each loop with dt in seconds.
     */
    public void update(double dt) {
        // --- PIDF position control when moving ---
        if (moving) {
            moveTimerS += dt;

            int currentPos = indexer.getCurrentPosition();
            int error = targetPosition - currentPos;

            double output;

            // Phase 1: short high-power "kick" to overcome static friction/spring
            if (kickPhase && moveTimerS < KICK_TIME_S) {
                output = Math.signum(error) * KICK_POWER;
            } else {
                // End of kick phase (if we were in it)
                kickPhase = false;

                // PIDF
                double errorD = (dt > 0) ? (error - lastError) / dt : 0.0;
                integral += error * dt;

                output = kP * error
                        + kI * integral
                        + kD * errorD
                        + kF * Math.signum(error);

                // Clamp output to [-basePower, basePower]
                output = clamp(output, -basePower, basePower);
            }

            indexer.setPower(output);
            lastError = error;

            // --- Stop condition with "hold in band" time ---
            if (Math.abs(error) <= POSITION_TOL) {
                withinTolTimerS += dt;
            } else {
                withinTolTimerS = 0.0;
            }

            if (withinTolTimerS >= WITHIN_TOL_HOLD_S) {
                moving = false;
                indexer.setPower(0.0);
                integral = 0.0;
                withinTolTimerS = 0.0;
            }
        }

        // --- Handle auto 3-ball sequence ---
        if (autoRunning) {
            timerS += dt;

            switch (autoStep) {
                case 0:
                    // wait a short delay before first advance
                    if (timerS > STEP_DELAY) {
                        timerS = 0;
                        autoStep = 1;
                        targetPosition += ticksPerSlot;
                        startMoveToTarget();
                    }
                    break;

                case 1:
                    // wait for first move to finish, then short delay
                    if (!moving && timerS > STEP_DELAY) {
                        timerS = 0;
                        autoStep = 2;
                        targetPosition += ticksPerSlot;
                        startMoveToTarget();
                    }
                    break;

                case 2:
                    // after second move and delay, close cam and finish
                    if (!moving && timerS > STEP_DELAY) {
                        setCamOpen(false);
                        autoRunning = false;
                    }
                    break;
            }
        }
    }

    private void startMoveToTarget() {
        moving = true;
        // Start a new "kick" phase for this move
        kickPhase = true;
        moveTimerS = 0.0;
        withinTolTimerS = 0.0;

        // clear PID state for a clean move
        integral = 0.0;
        lastError = targetPosition - indexer.getCurrentPosition();

        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // --- Helpers / accessors ---

    public boolean isMoving() { return moving; }

    public boolean isAutoRunning() { return autoRunning; }

    public void homeCam() {
        setCamOpen(false); // sends servo to camInitPos
    }

    public void setCamOpen(boolean open) {
        camOpen = open;
        camServo.setPosition(camOpen ? camOpenPos : camInitPos);
    }

    public boolean isCamOpen() { return camOpen; }

    public int getCurrentPosition() {
        return (indexer != null) ? indexer.getCurrentPosition() : 0;
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    private static double clamp(double val, double min, double max) {
        if (val < min) return min;
        if (val > max) return max;
        return val;
    }
}




