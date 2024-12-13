package frc.lib.generic.simulation.mechanisms;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.Logger;

/**
 * A Pose3d object that can change in rotation
 */
public class Component3d {
    private final String name;
    private final Translation3d poseTranslation;


    /**
     * create a 3d Component
     */
    public Component3d(String name, double rootX, double rootY, double rootZ) {
        this.name = name;
        this.poseTranslation = new Translation3d(rootX, rootY, rootZ);
    }

    /**
     * Updates the mechanism's angle and target angle, then logs the Pose3d object.
     */
    public void updateComponent(Rotation2d pitch, Rotation2d yaw,Translation3d newPosition) {
        Logger.recordOutput("Component3d/" + name,
                new Pose3d(poseTranslation.plus(newPosition),
                        new Rotation3d(0,
                                pitch.getRadians(),
                                yaw.getRadians())));
    }
}