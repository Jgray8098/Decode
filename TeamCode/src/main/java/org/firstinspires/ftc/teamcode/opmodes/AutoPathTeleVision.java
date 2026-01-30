package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.control.HeadingLockController;
import org.firstinspires.ftc.teamcode.mechanism.Flywheel;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.LimelightVisionFtc;
import org.firstinspires.ftc.teamcode.mechanism.Indexer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;

//@TeleOp(name = "AutoPathTeleVision")
public class AutoPathTeleVision extends OpMode {
    private final MecanumDrive drive = new MecanumDrive();

    // Tag IDs
    private static final int BLUE_GOAL_TID = 20;
    private static final int RED_GOAL_TID  = 24;

    // Pipelines
    private int bluePipe = 0;
    private int redPipe  = 1;

    // Heading lock tx setpoints
    private static final double TX_SETPOINT_CLOSE = 0.0;
    private static final double TX_SETPOINT_BLUE_LONG = +2;
    private static final double TX_SETPOINT_RED_LONG  = -2;

    // Auto-nav targets (Pedro coords)
    private static final Pose FAR_LAUNCH_RED_POSE  = new Pose(88, 20, Math.toRadians(68));
    private static final Pose FAR_LAUNCH_BLUE_POSE = new Pose(53, 21, Math.toRadians(106));
    private static final double CANCEL_STICK_THRESH = 0.12;

    // Field size (FTC centered -> Pedro corner mapping)
    private static final double FIELD_HALF_IN = 72.0;

    // Vision fusion tuning (gentle)
    private static final double VISION_FUSE_GAIN_XY = 0.12;
    private static final double VISION_FUSE_GAIN_H  = 0.18;

    private static final double VISION_MAX_JUMP_XY_IN = 30.0;
    private static final double VISION_MAX_JUMP_H_DEG = 25.0;

    // Snap/bootstrap
    private boolean poseTrusted = false;
    private boolean allowAutoSnap = true;
    private long lastSnapMs = 0;
    private static final long SNAP_COOLDOWN_MS = 800;
    private static final double SNAP_MAX_JUMP_IN = 200.0;

    // Sanity bounds (Pedro corner-origin field should be ~0..144)
    private static final double PEDRO_SANITY_MIN = -10.0;
    private static final double PEDRO_SANITY_MAX = 154.0;

    private Limelight3A limelight;
    private LimelightVisionFtc llVision;
    private HeadingLockController lockCtrl;

    // IMU (for MT2 yaw feed)
    private IMU imu;
    private boolean imuPresent = false;

    private int selectedTid;
    private int selectedPipe;

    private boolean prevRB1 = false;
    private boolean prevY1 = false;
    private boolean prevB1 = false;
    private boolean prevX1 = false;

    private long lastNs;

    private DcMotor intakeMotor;

    private Indexer indexer;
    private boolean prevLB2 = false;
    private boolean prevRB2 = false;
    private boolean prevB2  = false;
    private boolean jogMode = false;
    private boolean prevPS2 = false;

    private Flywheel flywheel;
    private boolean prevUp2 = false, prevDown2 = false;

    // Pedro auto-nav
    private Follower follower;
    private boolean autoNavActive = false;
    private PathChain autoNavPath = null;

    // Debug
    private boolean lastVisionApplied = false;
    private String fuseAction = "NONE";
    private String fuseXYSource = "none";
    private String fuseYawSource = "none";
    private Pose lastVisionPosePedro = null;

    @Override
    public void init() {
        drive.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);

        Pose startPose = safeStartPosePedro();
        follower.setStartingPose(startPose);
        follower.setPose(startPose);

        poseTrusted = (PoseStorage.lastPose != null);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        llVision = new LimelightVisionFtc(limelight);

        try {
            imu = hardwareMap.get(IMU.class, "imu");
            imuPresent = true;
        } catch (Exception ignored) {
            imuPresent = false;
        }

        selectedTid  = BLUE_GOAL_TID;
        selectedPipe = bluePipe;

        pushVisionConfig();
        llVision.start(100);

        HeadingLockController.Config cfg = new HeadingLockController.Config();
        lockCtrl = new HeadingLockController(llVision, null, cfg);
        lockCtrl.setDesiredTid(selectedTid);
        lockCtrl.setDesiredTxDeg(TX_SETPOINT_CLOSE);

        lastNs = System.nanoTime();

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        indexer = new Indexer(
                "Indexer", "camServo",
                1425, 3,
                0.7,
                0.65, 0.0
        );
        indexer.setHomeCamOnInit(false);
        indexer.init(hardwareMap);
        indexer.setStepDelay(0.15);

        flywheel = new Flywheel("flywheelRight", "flywheelLeft", "hoodServo");
        flywheel.init(hardwareMap);
        flywheel.setHoodStartPos(0.00);
        flywheel.setHoodPositions(0.15, 0.40);

        telemetry.addLine("[G1] LB=Hold Align, RB=Toggle Goal (BLUE↔RED)");
        telemetry.addLine("[G1] Y = AUTO-NAV to FAR LAUNCH pose (cancel by moving sticks)");
        telemetry.addLine("[G1] B = Cancel AUTO-NAV");
        telemetry.addLine("[G1] X = SNAP pose to Limelight (bootstrap)");
        telemetry.addLine("Manual drive = MecanumDrive ONLY. Pedro motors ONLY during auto-nav.");
        telemetry.addLine("IMPORTANT: In manual mode we call follower.update() FIRST (zero cmd), then drive.drive() LAST.");
        telemetry.addLine("bp->Pedro mapping: (xP = bpY+72, yP = -bpX+72), yawP = yaw-90");
    }

    @Override
    public void init_loop() {
        llVision.poll();
        updateLimelightImuYaw();

        // ---- CRITICAL: keep Pedro pose updating in init WITHOUT motor fighting ----
        updatePedroLocalizationSafe();   // follower.update() with zero output

        // Vision fusion after pose update
        applyLimelightPoseFusion();

        handleAllianceToggle();
        pushVisionConfig();
        updateHeadingTxSetpoint();

        telemetryAll(false);
        telemetry.update();
    }

    @Override
    public void start() {
        indexer.homeCam();
        indexer.syncToNextPocketForward(true);
        flywheel.enableHoodControl(true);

        // hard stop
        drive.drive(0, 0, 0);

        // make sure follower isn't mid-path
        stopFollowerNow();

        // one clean localization tick
        updatePedroLocalizationSafe();
    }

    @Override
    public void loop() {
        llVision.poll();
        updateLimelightImuYaw();

        long now = System.nanoTime();
        double dt = (now - lastNs) / 1e9;
        lastNs = now;

        handleAllianceToggle();
        pushVisionConfig();

        // Flywheel toggles
        boolean up2 = gamepad2.dpad_up;
        boolean down2 = gamepad2.dpad_down;
        if (up2 && !prevUp2) flywheel.toggleClose();
        if (down2 && !prevDown2) flywheel.toggleLong();
        prevUp2 = up2;
        prevDown2 = down2;

        updateHeadingTxSetpoint();
        flywheel.update(dt);

        boolean flywheelActive = flywheel.getTargetRpm() > 0.0;

        // Auto-nav trigger/cancel
        boolean y1 = gamepad1.y;
        if (y1 && !prevY1 && !autoNavActive) startAutoNavToFarLaunch();
        prevY1 = y1;

        boolean b1 = gamepad1.b;
        if (autoNavActive && b1 && !prevB1) cancelAutoNav();
        prevB1 = b1;

        // Manual SNAP (edge detect)
        boolean x1 = gamepad1.x;
        if (x1 && !prevX1) {
            poseTrusted = false; // forces snap path
        }
        prevX1 = x1;

        if (autoNavActive && driverIsCommandingSticks()) cancelAutoNav();

        if (autoNavActive) {
            // Pedro owns drivetrain while active
            follower.update();
            if (!follower.isBusy()) cancelAutoNav();
        } else {
            // ===== CRITICAL ORDER IN MANUAL MODE =====
            // 1) Update Pedro pose first, with ZERO follower output
            updatePedroLocalizationSafe();

            // 2) Apply fusion AFTER odom update (so odom doesn't instantly overwrite it)
            applyLimelightPoseFusion();

            // 3) Manual drive LAST so it always wins motor ownership
            // ===== KNOWN-GOOD SIGNS — DO NOT CHANGE =====
            double forward = -gamepad1.right_stick_y;
            double right   =  gamepad1.right_stick_x;
            double rotateDriver =  gamepad1.left_stick_x;
            // ============================================

            boolean lockHold = gamepad1.left_bumper;
            double omega = lockCtrl.update(dt, rotateDriver, lockHold, true);

            drive.drive(forward, right, omega);
        }

        // Intake
        double intakePower = 0.0;
        if (gamepad2.y)      intakePower = 1.0;
        else if (gamepad2.a) intakePower = -1.0;
        intakeMotor.setPower(intakePower);

        // Indexer
        boolean ps2 = gamepad2.ps;
        if (ps2 && !prevPS2) {
            jogMode = !jogMode;
            if (jogMode) indexer.enterJogMode();
            else         indexer.exitJogMode(true);
        }
        prevPS2 = ps2;

        if (jogMode) {
            indexer.jog(gamepad2.left_stick_x);
            telemetry.addLine("INDEXER JOG MODE: stick X to jog, PS to exit+reset");
            telemetryAll(true);
            telemetry.update();
            return;
        }

        boolean b2 = gamepad2.b;
        if (b2 && !prevB2 && flywheelActive && !indexer.isAutoRunning() && !indexer.isMoving()) {
            indexer.startAutoLaunchAllThreeContinuous();
        }
        prevB2 = b2;

        boolean lb2 = gamepad2.left_bumper;
        if (lb2 && !prevLB2 && flywheelActive && !indexer.isAutoRunning() && !indexer.isMoving()) {
            indexer.advanceOneSlot();
        }
        prevLB2 = lb2;

        boolean rb2 = gamepad2.right_bumper;
        if (rb2 && !prevRB2 && !indexer.isAutoRunning()) {
            indexer.setCamOpen(!indexer.isCamOpen());
        }
        prevRB2 = rb2;

        indexer.update(dt);

        telemetryAll(true);
        telemetry.update();
    }

    // ================= AUTO-NAV =================

    private void startAutoNavToFarLaunch() {
        Pose target = (selectedTid == RED_GOAL_TID) ? FAR_LAUNCH_RED_POSE : FAR_LAUNCH_BLUE_POSE;

        // Stop manual motors
        drive.drive(0, 0, 0);

        // Make sure pose is current before path starts
        updatePedroLocalizationSafe();
        applyLimelightPoseFusion();

        Pose cur = follower.getPose();
        if (cur == null) cur = safeStartPosePedro();

        autoNavPath = follower.pathBuilder()
                .addPath(new BezierLine(cur, target))
                .setLinearHeadingInterpolation(cur.getHeading(), target.getHeading())
                .build();

        follower.followPath(autoNavPath, true);
        autoNavActive = true;
    }

    private void cancelAutoNav() {
        stopFollowerNow();
        autoNavActive = false;
        autoNavPath = null;

        // Hard stop and return to manual
        drive.drive(0, 0, 0);
    }

    private void stopFollowerNow() {
        // Different Pedro builds have different names; try common ones safely.
        try { follower.getClass().getMethod("cancelFollowing").invoke(follower); } catch (Exception ignored) {}
        try { follower.getClass().getMethod("cancel").invoke(follower); } catch (Exception ignored) {}
        try { follower.getClass().getMethod("stop").invoke(follower); } catch (Exception ignored) {}

        // IMPORTANT: force follower into a benign state so follower.update() won’t try to follow a path
        try { follower.startTeleopDrive(); } catch (Exception ignored) {}
        try { follower.setTeleOpDrive(0, 0, 0, true); } catch (Exception ignored) {}
    }

    private boolean driverIsCommandingSticks() {
        double f = Math.abs(-gamepad1.right_stick_y);
        double s = Math.abs( gamepad1.right_stick_x);
        double r = Math.abs( gamepad1.left_stick_x);
        return (f > CANCEL_STICK_THRESH) || (s > CANCEL_STICK_THRESH) || (r > CANCEL_STICK_THRESH);
    }

    // ================= Localization SAFE (updates pose; zero follower output) =================
    // This is the replacement for the reflection approach.
    private void updatePedroLocalizationSafe() {
        // Force follower into teleop mode and command zero output BEFORE update.
        try { follower.startTeleopDrive(); } catch (Exception ignored) {}
        try { follower.setTeleOpDrive(0, 0, 0, true); } catch (Exception ignored) {}

        // Update pose estimate (this may still write motor power internally,
        // but since we commanded zero, output should be zero).
        follower.update();
    }

    // ================= Limelight IMU feed =================

    private void updateLimelightImuYaw() {
        if (!imuPresent) return;
        try {
            YawPitchRollAngles o = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(o.getYaw(AngleUnit.DEGREES));
        } catch (Exception ignored) {}
    }

    // ================= Vision Fusion =================

    private void applyLimelightPoseFusion() {
        lastVisionApplied = false;
        fuseAction = "NONE";
        fuseXYSource = "none";
        fuseYawSource = "none";
        lastVisionPosePedro = null;

        Pose cur = follower.getPose();
        if (cur == null) return;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        Pose3D bp = null;
        Pose3D mt2 = null;

        try { bp  = result.getBotpose(); } catch (Exception ignored) {}
        try { mt2 = result.getBotpose_MT2(); } catch (Exception ignored) {}

        if (bp == null && mt2 == null) return;

        // Read bp
        Double xBp = null, yBp = null, yawBpDeg = null;
        if (bp != null) {
            try {
                Position pIn = bp.getPosition().toUnit(DistanceUnit.INCH);
                xBp = pIn.x; yBp = pIn.y;
                yawBpDeg = bp.getOrientation().getYaw(AngleUnit.DEGREES);
            } catch (Exception ignored) {}
        }

        // Read mt2
        Double xMt2 = null, yMt2 = null, yawMt2Deg = null;
        if (mt2 != null) {
            try {
                Position pIn = mt2.getPosition().toUnit(DistanceUnit.INCH);
                xMt2 = pIn.x; yMt2 = pIn.y;
                yawMt2Deg = mt2.getOrientation().getYaw(AngleUnit.DEGREES);
            } catch (Exception ignored) {}
        }

        // Choose translation (prefer bp)
        Double xUse = null, yUse = null;
        if (xBp != null && yBp != null) {
            xUse = xBp; yUse = yBp;
            fuseXYSource = "bp";
        } else if (xMt2 != null && yMt2 != null) {
            xUse = xMt2; yUse = yMt2;
            fuseXYSource = "mt2";
        } else {
            return;
        }

        // Choose yaw (prefer mt2)
        double yawUseDeg;
        if (yawMt2Deg != null) {
            yawUseDeg = yawMt2Deg;
            fuseYawSource = "mt2";
        } else if (yawBpDeg != null) {
            yawUseDeg = yawBpDeg;
            fuseYawSource = "bp";
        } else {
            yawUseDeg = Math.toDegrees(cur.getHeading());
            fuseYawSource = "cur";
        }

        // Empirically correct bp -> Pedro mapping:
        // pedroX = bpY + 72
        // pedroY = -bpX + 72
        // yawPedro = yaw - 90
        double pedroX = yUse + FIELD_HALF_IN;
        double pedroY = -xUse + FIELD_HALF_IN;
        double yawPedroDeg = yawUseDeg - 90.0;

        Pose visionPosePedro = new Pose(
                pedroX,
                pedroY,
                Math.toRadians(yawPedroDeg),
                PedroCoordinates.INSTANCE
        );

        lastVisionPosePedro = visionPosePedro;

        if (!isSanePedroXY(visionPosePedro.getX(), visionPosePedro.getY())) {
            fuseAction = "REJECT_SANITY";
            return;
        }

        double dx = visionPosePedro.getX() - cur.getX();
        double dy = visionPosePedro.getY() - cur.getY();
        double dxy = Math.hypot(dx, dy);

        double dHeadDeg = Math.toDegrees(Math.abs(normDelta(visionPosePedro.getHeading() - cur.getHeading())));
        boolean haveHeading = dHeadDeg <= VISION_MAX_JUMP_H_DEG;

        long nowMs = System.currentTimeMillis();
        boolean canSnap = (nowMs - lastSnapMs) > SNAP_COOLDOWN_MS;
        boolean wantSnap = (gamepad1.x) || (allowAutoSnap && !poseTrusted);

        if (wantSnap && canSnap && dxy <= SNAP_MAX_JUMP_IN) {
            follower.setPose(visionPosePedro);
            poseTrusted = true;
            lastSnapMs = nowMs;
            lastVisionApplied = true;
            fuseAction = "SNAP";
            return;
        }

        if (dxy > VISION_MAX_JUMP_XY_IN) {
            fuseAction = "REJECT_JUMP";
            return;
        }

        double newX = cur.getX() + dx * VISION_FUSE_GAIN_XY;
        double newY = cur.getY() + dy * VISION_FUSE_GAIN_XY;

        double newH = cur.getHeading();
        if (haveHeading) {
            double dHead = normDelta(visionPosePedro.getHeading() - cur.getHeading());
            newH = cur.getHeading() + dHead * VISION_FUSE_GAIN_H;
        }

        follower.setPose(new Pose(newX, newY, newH));
        lastVisionApplied = true;
        fuseAction = "FUSE";
    }

    private boolean isSanePedroXY(double x, double y) {
        return (x >= PEDRO_SANITY_MIN && x <= PEDRO_SANITY_MAX && y >= PEDRO_SANITY_MIN && y <= PEDRO_SANITY_MAX);
    }

    private static double normDelta(double rad) {
        double deg = Math.toDegrees(rad);
        double d = (deg + 540.0) % 360.0 - 180.0;
        return Math.toRadians(d);
    }

    // ================= Pose / Vision config =================

    private Pose safeStartPosePedro() {
        Pose p = PoseStorage.lastPose;
        if (p == null) {
            return new Pose(0, 0, 0, PedroCoordinates.INSTANCE);
        }
        return p;
    }

    private void updateHeadingTxSetpoint() {
        boolean isBlue = (selectedTid == BLUE_GOAL_TID);
        boolean isLong = (flywheel.getState() == Flywheel.State.LONG);

        double txSetpoint = TX_SETPOINT_CLOSE;
        if (isLong) txSetpoint = isBlue ? TX_SETPOINT_BLUE_LONG : TX_SETPOINT_RED_LONG;

        lockCtrl.setDesiredTxDeg(txSetpoint);
    }

    private void handleAllianceToggle() {
        boolean rb1 = gamepad1.right_bumper;
        if (rb1 && !prevRB1) {
            selectedTid = (selectedTid == BLUE_GOAL_TID) ? RED_GOAL_TID : BLUE_GOAL_TID;
            selectedPipe = (selectedTid == BLUE_GOAL_TID) ? bluePipe : redPipe;

            lockCtrl.setDesiredTid(selectedTid);
            llVision.setPreferredTid(selectedTid);

            if (autoNavActive) cancelAutoNav();
        }
        prevRB1 = rb1;
    }

    private void pushVisionConfig() {
        limelight.pipelineSwitch(selectedPipe);
        llVision.setPipeline(selectedPipe);
        llVision.setPreferredTid(selectedTid);
    }

    // ================= Telemetry =================

    private void telemetryAll(boolean includeFlywheel) {
        telemetry.addData("Alliance/Goal", (selectedTid == RED_GOAL_TID) ? "RED(24)" : "BLUE(20)");
        telemetry.addData("Active Pipe", selectedPipe);

        telemetry.addData("HasTarget", llVision.hasTarget());
        telemetry.addData("tid", llVision.getTid());
        telemetry.addData("tx", "%.2f", llVision.getTxDeg());
        telemetry.addData("TxSet", "%.1f", lockCtrl.getDesiredTxDeg());

        if (imuPresent) {
            try {
                YawPitchRollAngles o = imu.getRobotYawPitchRollAngles();
                telemetry.addData("IMU Yaw", "%.1f", o.getYaw(AngleUnit.DEGREES));
            } catch (Exception e) {
                telemetry.addData("IMU Yaw", "err");
            }
        } else telemetry.addData("IMU Yaw", "no IMU");

        LLResult r = limelight.getLatestResult();
        telemetry.addData("LL valid", (r != null && r.isValid()));

        if (r != null && r.isValid()) {
            // bp
            try {
                Pose3D bp = r.getBotpose();
                if (bp != null) {
                    Position pIn = bp.getPosition().toUnit(DistanceUnit.INCH);
                    double yaw = bp.getOrientation().getYaw(AngleUnit.DEGREES);
                    telemetry.addData("LL bp IN", "(%.1f, %.1f)", pIn.x, pIn.y);
                    telemetry.addData("LL bp Yaw", "%.1f", yaw);
                } else telemetry.addData("LL bp", "null");
            } catch (Exception e) {
                telemetry.addData("LL bp err", e.getClass().getSimpleName());
            }

            // mt2
            try {
                Pose3D mt2 = r.getBotpose_MT2();
                if (mt2 != null) {
                    Position pIn = mt2.getPosition().toUnit(DistanceUnit.INCH);
                    double yaw = mt2.getOrientation().getYaw(AngleUnit.DEGREES);
                    telemetry.addData("LL MT2 IN", "(%.1f, %.1f)", pIn.x, pIn.y);
                    telemetry.addData("LL MT2 Yaw", "%.1f", yaw);
                } else telemetry.addData("LL MT2", "null");
            } catch (Exception e) {
                telemetry.addData("LL MT2 err", e.getClass().getSimpleName());
            }
        }

        if (lastVisionPosePedro != null) {
            telemetry.addData("vision->Pedro", "(%.1f, %.1f, %.1f°)",
                    lastVisionPosePedro.getX(), lastVisionPosePedro.getY(),
                    Math.toDegrees(lastVisionPosePedro.getHeading()));
        } else {
            telemetry.addData("vision->Pedro", "null");
        }

        Pose p = follower.getPose();
        telemetry.addData("PedroPose", (p != null)
                ? String.format("(%.1f, %.1f, %.1f°)", p.getX(), p.getY(), Math.toDegrees(p.getHeading()))
                : "NULL");

        telemetry.addData("Fuse xySrc", fuseXYSource);
        telemetry.addData("Fuse yawSrc", fuseYawSource);
        telemetry.addData("Fuse action", fuseAction);
        telemetry.addData("PoseTrusted", poseTrusted);

        telemetry.addData("AutoNav", autoNavActive);

        if (includeFlywheel) {
            telemetry.addData("Flywheel State", flywheel.getState());
            telemetry.addData("Flywheel Target RPM", "%.0f", flywheel.getTargetRpm());
            telemetry.addData("Flywheel Right RPM", "%.0f", flywheel.getMeasuredRightRpm());
            telemetry.addData("Flywheel Left  RPM", "%.0f", flywheel.getMeasuredLeftRpm());
        }

        telemetry.addData("ManualMode PedroUpdate", "follower.update() zero cmd FIRST; drive LAST");
    }
}
