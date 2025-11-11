package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Simplified Indexer:
 *  - advanceOneSlot() for manual use
 *  - autoLaunchAllThree(): open cam, advance twice, close cam, done
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

    // timing (seconds)
    private static final double STEP_DELAY = 0.15;
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

    public void init(HardwareMap hw) {
        indexer = hw.get(DcMotorEx.class, indexerMotorName);
        camServo = hw.get(Servo.class, camServoName);

        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ticksPerSlot = ticksPerRev / Math.max(1, slots);
        targetPosition = 0;
        moving = false;
        camOpen = false;
        // Only move servo during init if allowed
        if (homeCamOnInit) {
            camServo.setPosition(camInitPos);
        }
    }

    /** Manual single advance. */
    public void advanceOneSlot() {
        if (moving || autoRunning) return;
        targetPosition += ticksPerSlot;
        runToTarget();
    }

    /** Begin automatic 3-ball launch (open cam + 2 advances). */
    public void startAutoLaunchAllThree() {
        if (autoRunning || moving) return;
        autoRunning = true;
        autoStep = 0;
        timerS = 0;
        setCamOpen(true); // launch first ball
    }

    public void update(double dt) {
        // Manage motion end
        if (moving && !indexer.isBusy()) {
            indexer.setPower(0.0);
            indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            moving = false;
        }

        // Handle auto sequence
        if (autoRunning) {
            timerS += dt;

            switch (autoStep) {
                case 0:
                    // wait a short delay before first advance
                    if (timerS > STEP_DELAY) {
                        timerS = 0;
                        autoStep = 1;
                        targetPosition += ticksPerSlot;
                        runToTarget();
                    }
                    break;

                case 1:
                    // wait for first move to finish, then short delay
                    if (!moving && timerS > STEP_DELAY) {
                        timerS = 0;
                        autoStep = 2;
                        targetPosition += ticksPerSlot;
                        runToTarget();
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