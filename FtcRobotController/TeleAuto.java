package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.HeadingLockController;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive; // <-- your class
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.vision.LimelightVisionFtc;

@TeleOp(name = "TeleAuto")
public class TeleAuto extends OpMode {

    // ===== Field targets (Pedro coordinates) =====
    private static final Pose FAR_LAUNCH_RED_POSE  = new Pose(88, 20, Math.toRadians(68));
    private static final Pose FAR_LAUNCH_BLUE_POSE = new Pose(53, 21, Math.toRadians(106));

    // Cancel auto-nav if driver moves sticks beyond this
    private static final double CANCEL_STICK_THRESH = 0.10;

    // ===== Goal tags =====
    private static final int BLUE_GOAL_TID = 20;
    private static final int RED_GOAL_TID  = 24;

    // ===== Pipelines (set these to whatever you KNOW worked before) =====
    // If your "old working" setup used different pipeline numbers, change these two constants.
    private static final int PIPE_BLUE = 0;
    private static final int PIPE_RED  = 1;

    // ===== Heading lock tx setpoints =====
    private static final double TX_SETPOINT_CLOSE = 0.0;
    private static final double TX_SETPOINT_BLUE_LONG = +3.0;
    private static final double TX_SETPOINT_RED_LONG  = -3.0;

    // ===== Manual drive =====
    private final MecanumDrive mecanum = new MecanumDrive();

    // ===== Pedro (auto only) =====
    private Follower follower;
    private boolean autoNavActive = false;
    private PathChain autoNavPath = null;

    // ===== Limelight + heading lock =====
    private Limelight3A limelight;
    private LimelightVisionFtc llVision;
    private HeadingLockController lockCtrl;

    // ===== Alliance select =====
    private int selectedTid = BLUE_GOAL_TID;

    // Edge detect
    private boolean prevRB1 = false;
    private boolean prevY1  = false;

    // Timing
    private long lastNs = 0;

    @Override
    public void init() {
        // Manual drive motors
        mecanum.init(hardwareMap);

        // Pedro follower for auto-nav only
        follower = Constants.createFollower(hardwareMap);

        Pose startPose = (PoseStorage.lastPose != null) ? PoseStorage.lastPose : new Pose();
        follower.setStartingPose(startPose);
        follower.update();

        // Limelight + wrapper
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        llVision = new LimelightVisionFtc(limelight);

        // Start on blue by default
        applyAllianceVisionConfig();

        // Heading lock controller (your ORIGINAL good one)
        HeadingLockController.Config cfg = new HeadingLockController.Config();
        lockCtrl = new HeadingLockController(llVision, null, cfg);
        lockCtrl.setDesiredTid(selectedTid);
        lockCtrl.setDesiredTxDeg(TX_SETPOINT_CLOSE);

        // Start limelight polling (wrapper) + camera
        limelight.start();
        llVision.start(100);

        lastNs = System.nanoTime();

        telemetry.addLine("RESET TeleOp:");
        telemetry.addLine(" - Manual drive = your MecanumDrive class");
        telemetry.addLine(" - Heading lock = your original HeadingLockController");
        telemetry.addLine(" - Auto-nav = Pedro follower ONLY when G1.Y pressed");
        telemetry.addLine("G1.RB toggles alliance/tag/pipeline. G1.Y auto-nav. Move sticks to cancel.");
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        double dt = (now - lastNs) / 1e9;
        lastNs = now;

        // Keep Pedro pose estimate updated every loop (safe, we overwrite motor power in manual mode)
        follower.update();

        // Poll Limelight wrapper for tx/tid/tv
        llVision.poll();

        // Alliance toggle
        boolean rb1 = gamepad1.right_bumper;
        if (rb1 && !prevRB1) {
            selectedTid = (selectedTid == BLUE_GOAL_TID) ? RED_GOAL_TID : BLUE_GOAL_TID;
            applyAllianceVisionConfig();
        }
        prevRB1 = rb1;

        // Auto-nav start
        boolean y1 = gamepad1.y;
        if (y1 && !prevY1) {
            startAutoNavToFarLaunch();
        }
        prevY1 = y1;

        // Cancel auto-nav if driver moves sticks
        if (autoNavActive && driverIsCommandingSticks()) {
            cancelAutoNav();
        }

        // If auto-nav active, let Pedro own motors
        if (autoNavActive) {
            if (!follower.isBusy()) {
                cancelAutoNav();
            }

            telemetry.addData("MODE", "AUTO-NAV");
            telemetry.addData("Target", (selectedTid == RED_GOAL_TID) ? "RED FAR" : "BLUE FAR");
            telemetry.addData("FollowerBusy", follower.isBusy());
            telemetry.update();
            return;
        }

        // ===== MANUAL MODE (MecanumDrive is the ONLY thing commanding motors) =====
        // Use your original stick mapping that you KNOW is correct.
        double forward = -gamepad1.left_stick_y;
        double right   = -gamepad1.left_stick_x;
        double rotate  = -gamepad1.right_stick_x;

        boolean lockHold = gamepad1.left_bumper;

        // Update setpoint (close vs long) — swap this to your actual flywheel state if needed
        // For now: hold X for "long" just to prove the logic works.
        boolean longShot = gamepad1.x;
        double txSetpoint = TX_SETPOINT_CLOSE;
        if (longShot) {
            txSetpoint = (selectedTid == BLUE_GOAL_TID) ? TX_SETPOINT_BLUE_LONG : TX_SETPOINT_RED_LONG;
        }
        lockCtrl.setDesiredTxDeg(txSetpoint);
        lockCtrl.setDesiredTid(selectedTid);

        // Heading lock: while held, DO NOT feed driver rotate into controller
        double omegaCmd;
        if (lockHold) {
            omegaCmd = lockCtrl.update(dt, 0.0, true, true);
        } else {
            omegaCmd = rotate;
        }

        // >>> IMPORTANT: we are NOT changing your manual rotation sign here.
        // Your MecanumDrive expects rotate to be the same sign you were already using.
        mecanum.drive(forward, right, omegaCmd);

        // Telemetry
        Pose p = follower.getPose();
        telemetry.addData("MODE", "MANUAL");
        telemetry.addData("Alliance", (selectedTid == RED_GOAL_TID) ? "RED(24)" : "BLUE(20)");
        telemetry.addData("HasTarget", llVision.hasTarget());
        telemetry.addData("tid", llVision.getTid());
        telemetry.addData("tx", "%.2f", llVision.getTxDeg());
        telemetry.addData("TxSetpoint", "%.2f", lockCtrl.getDesiredTxDeg());
        telemetry.addData("PedroPose", "(%.1f, %.1f, %.1f°)", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        telemetry.update();
    }

    // ===== Auto-nav helpers =====

    private void startAutoNavToFarLaunch() {
        Pose target = (selectedTid == RED_GOAL_TID) ? FAR_LAUNCH_RED_POSE : FAR_LAUNCH_BLUE_POSE;

        Pose cur = follower.getPose();
        autoNavPath = follower.pathBuilder()
                .addPath(new BezierLine(cur, target))
                .setLinearHeadingInterpolation(cur.getHeading(), target.getHeading())
                .build();

        follower.followPath(autoNavPath, true);
        autoNavActive = true;
    }

    private void cancelAutoNav() {
        // Stop follower output immediately
        follower.startTeleopDrive();
        follower.setTeleOpDrive(0, 0, 0, true);

        autoNavActive = false;
        autoNavPath = null;
    }

    private boolean driverIsCommandingSticks() {
        double f = Math.abs(gamepad1.left_stick_y);
        double s = Math.abs(gamepad1.left_stick_x);
        double r = Math.abs(gamepad1.right_stick_x);
        return (f > CANCEL_STICK_THRESH) || (s > CANCEL_STICK_THRESH) || (r > CANCEL_STICK_THRESH);
    }

    // ===== Vision config per alliance =====

    private void applyAllianceVisionConfig() {
        if (selectedTid == BLUE_GOAL_TID) {
            limelight.pipelineSwitch(PIPE_BLUE);
            llVision.setPipeline(PIPE_BLUE);
            llVision.setPreferredTid(BLUE_GOAL_TID);
        } else {
            limelight.pipelineSwitch(PIPE_RED);
            llVision.setPipeline(PIPE_RED);
            llVision.setPreferredTid(RED_GOAL_TID);
        }
        // Also reset heading controller so it doesn't carry history across alliance switch
        if (lockCtrl != null) lockCtrl.reset();
    }
}

/**
 * Shared pose memory:
 * - At end of autonomous: PoseStorage.lastPose = follower.getPose();
 */
class PoseStorage {
    public static Pose lastPose = null;
}
