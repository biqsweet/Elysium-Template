package frc.lib.generic.simulation.mechanisms;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.Logger;

/**
 * A Pose3d object that can change in position
 */
public class Object3d {
    private final String name;
    private final Translation3d poseTranslation;
    private final Rotation3d poseRotation;


    /**
     * create a 3d object
     */
    public Object3d(String name, double rootX, double rootY, double rootZ) {
        this.name = name;
        this.poseTranslation = new Translation3d(rootX, rootY, rootZ);
        this.poseRotation = new Rotation3d(0, 0, 0);
    }

    /**
     * Updates the object's position, then logs the Pose3d object.
     */
    public void updateObject(Translation3d position) {
        Logger.recordOutput("Pose3d/" + name,
                new Pose3d(poseTranslation.plus(position), poseRotation));
    }
}