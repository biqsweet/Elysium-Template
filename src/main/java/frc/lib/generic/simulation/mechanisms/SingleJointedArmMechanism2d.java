package frc.lib.generic.simulation.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import org.littletonrobotics.junction.Logger;

import static frc.lib.generic.simulation.mechanisms.MechanismConstants.*;
import static frc.lib.generic.simulation.mechanisms.MechanismUtilities.getRoot;

public class SingleJointedArmMechanism2d {
    private final String name;
    private final Mechanism2d armMechanism;
    private final MechanismRoot2d root;
    private MechanismLigament2d
            currentAngleLigament,
            targetAngleLigament;

    public SingleJointedArmMechanism2d(String name, double armLength, Rotation2d minimumAngle, Rotation2d maximumAngle) {
        this.name = name;
        this.armMechanism = new Mechanism2d(DEFAULT_CANVAS_WIDTH, DEFAULT_CANVAS_HEIGHT);
        this.root = getRoot("armRoot", armMechanism);

        createCurrent(armLength, minimumAngle);
        createTarget(armLength, minimumAngle);
    }

    public void updateCurrentMechanismAngle(Rotation2d newAngle) {
        currentAngleLigament.setAngle(newAngle);
        Logger.recordOutput(name, armMechanism);
    }

    public void updateTargetMechanismAngle(Rotation2d newTargetAngle) {
        targetAngleLigament.setAngle(newTargetAngle);
        Logger.recordOutput(name, armMechanism);
    }

    public Mechanism2d getMechanism() {
        return armMechanism;
    }

    private void createCurrent(double armLength, Rotation2d minimumAngle) {
        currentAngleLigament = new MechanismLigament2d("armLigament", armLength, minimumAngle.getDegrees(), DEFAULT_LINE_WIDTH, BLUE);
        root.append(currentAngleLigament);
    }

    private void createTarget(double armLength, Rotation2d minimumAngle) {
        targetAngleLigament = new MechanismLigament2d("targetArmLigament", armLength, minimumAngle.getDegrees(), DEFAULT_LINE_WIDTH, GRAY);
        root.append(targetAngleLigament);
    }
}