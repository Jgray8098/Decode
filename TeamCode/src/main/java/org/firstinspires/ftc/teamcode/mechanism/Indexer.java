package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Indexer mechanism that encapsulates:
 *  - A slotted index wheel driven by a motor with encoder
 *  - A cam servo with simple open/closed positions
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

    private int ticksPerSlot;
    private int targetPosition = 0;
    private boolean moving = false;
    private boolean camOpen = false;

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

    public void init(HardwareMap hw) {
        indexer = hw.get(DcMotorEx.class, indexerMotorName);
        camServo = hw.get(Servo.class, camServoName);

        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ticksPerSlot = ticksPerRev / Math.max(1, slots);
        targetPosition = 0;
        moving = false;

        camServo.setPosition(camInitPos);
        camOpen = false;
    }

    public void advanceOneSlot() {
        if (moving) return;
        targetPosition += ticksPerSlot;
        indexer.setTargetPosition(targetPosition);
        indexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexer.setPower(indexerPower);
        moving = true;
    }

    public void update() {
        if (!moving) return;
        int posErr = targetPosition - indexer.getCurrentPosition();
        boolean atTarget = Math.abs(posErr) <= POSITION_TOL || !indexer.isBusy();
        if (atTarget) {
            indexer.setPower(0.0);
            indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            moving = false;
        }
    }

    public boolean isMoving() { return moving; }
    public int getCurrentPosition() { return indexer.getCurrentPosition(); }
    public int getTargetPosition() { return targetPosition; }
    public int getNextTargetPosition() { return targetPosition + ticksPerSlot; }

    public void setCamOpen(boolean open) {
        camOpen = open;
        camServo.setPosition(camOpen ? camOpenPos : camInitPos);
    }

    public void toggleCam() { setCamOpen(!camOpen); }
    public boolean isCamOpen() { return camOpen; }
}
