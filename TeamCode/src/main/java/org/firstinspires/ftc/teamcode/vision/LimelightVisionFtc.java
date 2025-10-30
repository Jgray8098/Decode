package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.control.HeadingLockController;
import java.util.List;

/** Limelight (FTC API) adapter that prefers a selected tag ID and exposes tx/tid/valid. */
public class LimelightVisionFtc implements HeadingLockController.Vision {

    private final Limelight3A limelight;
    private volatile Integer preferredTid = null;

    private volatile double tx = 0.0;
    private volatile boolean tv = false;
    private volatile int tid = -1;
    private volatile long lastUpdateMs = 0;

    public LimelightVisionFtc(Limelight3A limelight) { this.limelight = limelight; }

    public void start(int pollHz) { limelight.setPollRateHz(pollHz); limelight.start(); }
    public void setPreferredTid(Integer tidOrNull) { this.preferredTid = tidOrNull; }
    public void setPipeline(int index) { limelight.pipelineSwitch(index); }

    public void poll() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) { tv = false; return; }

        if (preferredTid != null) {
            List<FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null) {
                for (FiducialResult f : fiducials) {
                    if (f.getFiducialId() == preferredTid) {
                        tx = f.getTargetXDegrees();
                        tid = f.getFiducialId();
                        tv  = true;
                        lastUpdateMs = System.currentTimeMillis();
                        return;
                    }
                }
            }
            tv = false;
        } else {
            tx = result.getTx();
            List<FiducialResult> fiducials = result.getFiducialResults();
            tid = (fiducials != null && !fiducials.isEmpty()) ? fiducials.get(0).getFiducialId() : -1;
            tv  = true;
            lastUpdateMs = System.currentTimeMillis();
        }
    }

    @Override public boolean hasTarget()   { return tv; }
    @Override public double  getTxDeg()    { return tx; }
    @Override public int     getTid()      { return tid; }
    @Override public long    lastUpdateMs(){ return lastUpdateMs; }
}
