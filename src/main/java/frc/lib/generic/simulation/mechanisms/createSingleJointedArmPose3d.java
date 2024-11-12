package frc.lib.generic.simulation.mechanisms;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotContainer.ARM;
import static frc.robot.RobotContainer.TURRET;

/**
 * A Pose3d object to display the current angle and the target angle of a single jointed arm.
 */
public class createSingleJointedArmPose3d {
    private final String name;
    private final double rootX;
    private final double rootY;
    private final double rootZ;

    /**
     * create a 3d pose with
     */
    public createSingleJointedArmPose3d(String name, double rootX, double rootY, double rootZ) {
        this.name = name;
        this.rootX = rootX;
        this.rootY = rootY;
        this.rootZ = rootZ;
    }

    private static Rotation3d getRotation3d() {
        return new Rotation3d(
                0.0,
                ARM.getCurrentArmPosition().getRadians(),
                TURRET.getCurrentTurretPosition().getRadians()
        );
    }

    /**
     * Updates the mechanism's angle and target angle, then logs the Pose3d object.
     *
     * @param currentAngle the current angle
     */
    public void updateMechanism(Rotation2d currentAngle) {
        Logger.recordOutput("Pose3d/" + name, new Pose3d(rootX, rootY, rootZ, getRotation3d()));
    }
}
