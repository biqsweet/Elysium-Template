package frc.lib.generic.simulation.mechanisms;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.Logger;

/**
 * A Pose3d object that can change in rotation
 */
public class Component3d { //todo: better name V
    private final String name;
    private final Translation3d poseTranslation;
    //todo: consider splatting these together V


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
    public void updateComponent(Rotation2d pitch, Rotation2d yaw) {
        Logger.recordOutput("Pose3d/" + name,
                new Pose3d(poseTranslation,
                        new Rotation3d(0,
                                pitch.getRadians(),
                                yaw.getRadians())));
    }
    //todo: goofy ahh conventions bruh. fix ur spacings. Create the translation part of the pose3d only once, as these DON'T change throughout the class. V
}