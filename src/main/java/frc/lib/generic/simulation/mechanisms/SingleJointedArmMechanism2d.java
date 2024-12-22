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

    public SingleJointedArmMechanism2d(String name, double armLength, Rotation2d defaultAngle) {
        this.name = name;
        this.armMechanism = new Mechanism2d(DEFAULT_CANVAS_WIDTH, DEFAULT_CANVAS_HEIGHT);
        this.root = getRoot("armRoot", armMechanism);

        createCurrentLigaments(armLength, defaultAngle);
        createTargetLigaments(armLength, defaultAngle);
    }

    public void updateCurrentAngle(Rotation2d angle) {
        currentAngleLigament.setAngle(angle);
        Logger.recordOutput(name, armMechanism);
    }

    public void updateTargetAngle(Rotation2d targetAngle) {
        targetAngleLigament.setAngle(targetAngle);
        Logger.recordOutput(name, armMechanism);
    }

    public Mechanism2d getMechanism() {
        return armMechanism;
    }

    private void createCurrentLigaments(double armLength, Rotation2d defaultAngle) {
        currentAngleLigament = new MechanismLigament2d("armLigament", armLength, defaultAngle.getDegrees(), DEFAULT_LINE_WIDTH, BLUE);
        root.append(currentAngleLigament);
    }

    private void createTargetLigaments(double armLength, Rotation2d defaultAngle) {
        targetAngleLigament = new MechanismLigament2d("targetArmLigament", armLength, defaultAngle.getDegrees(), DEFAULT_LINE_WIDTH, GRAY);
        root.append(targetAngleLigament);
    }
}