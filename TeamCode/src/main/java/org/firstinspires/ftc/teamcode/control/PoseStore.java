package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

public class PoseStore {
    private static final String FILE_NAME = "last_pose.txt";

    private static File file() {
        return AppUtil.getInstance().getSettingsFile(FILE_NAME);
    }

    /** Save pose (x,y,headingRad) to persistent storage. */
    public static void savePose(Pose p) {
        if (p == null) return;
        String s = String.format(Locale.US, "%.6f,%.6f,%.8f",
                p.getX(), p.getY(), p.getHeading());
        ReadWriteFile.writeFile(file(), s);
    }

    /** Load pose from storage. Returns null if not found/invalid. */
    public static Pose loadPoseOrNull() {
        try {
            String s = ReadWriteFile.readFile(file()).trim();
            if (s.isEmpty()) return null;
            String[] parts = s.split(",");
            if (parts.length < 3) return null;
            double x = Double.parseDouble(parts[0]);
            double y = Double.parseDouble(parts[1]);
            double h = Double.parseDouble(parts[2]);
            return new Pose(x, y, h);
        } catch (Exception ignored) {
            return null;
        }
    }

    /** Optional: clear pose file. */
    public static void clear() {
        ReadWriteFile.writeFile(file(), "");
    }
}

