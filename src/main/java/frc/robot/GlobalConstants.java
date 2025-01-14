package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class GlobalConstants {
    public static final BuiltInAccelerometer ROBORIO_ACCELEROMETER = new BuiltInAccelerometer();

    public static final double ROBOT_PERIODIC_LOOP_TIME = 0.02;

    public static final double ODOMETRY_FREQUENCY_HERTZ = 200.0;
    public static final Lock FASTER_THREAD_LOCK = new ReentrantLock();

    public static final boolean IS_TUNING_MODE = true;

    public static final boolean SHOULD_WRITE_LOGS = false; //for when the roborio doesn't have enough space...
    public static final boolean SHOULD_DISPLAY_MECHANISMS = true;

    public static final Mode CURRENT_MODE = Mode.REAL;

    public enum Mode {
        REAL, SIMULATION, REPLAY
    }

    public static final double GRAVITY = 9.80665;
    public static final double MINIMUM_ACCELERATION_FOR_COLLISION = 25; //todo: Tune

    public static final Translation3d HUB_POSITION = new Translation3d(8, 4,2);

    public static final double FIELD_LENGTH_METRES = 14.56;
    public static final double GRAVITY_FORCE = 9.806;

    public static final double VOLTAGE_COMPENSATION_SATURATION = 12;

    public static final Pose3d RED_SPEAKER = new Pose3d(new Translation3d(-0.0381+14.56,5.547868, 1.9), new Rotation3d());
    public static final Pose3d BLUE_SPEAKER = new Pose3d(new Translation3d(-0.0381,5.547868, 1.9), new Rotation3d());
//            TAG_ID_TO_POSE.get(7).plus(new Transform3d(new Translation3d(0.2, 0.0, 1.3), new Rotation3d()));
}
