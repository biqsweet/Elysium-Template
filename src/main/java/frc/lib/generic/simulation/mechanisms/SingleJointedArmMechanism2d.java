package frc.lib.generic.simulation.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;


public class SingleJointedArmMechanism2d {
    private final Mechanism2d armMechanism;
    private final MechanismLigament2d currentAngleLigament;
    private final MechanismLigament2d targetAngleLigament;
    private final MechanismRoot2d root;

    public SingleJointedArmMechanism2d(double armLengthMeters, Rotation2d minimumAngle, Rotation2d maximumAngle) {
        this.armMechanism = new Mechanism2d(10, 10);

        this.root = armMechanism.getRoot("armMechanism", 5, 5);

        this.currentAngleLigament = new MechanismLigament2d("armLigament", armLengthMeters, minimumAngle.getDegrees(), 3, new Color8Bit(0, 0, 255));
        this.targetAngleLigament = new MechanismLigament2d("targetAngleLigament", armLengthMeters, minimumAngle.getDegrees(), 3, new Color8Bit(144, 144, 144));

        root.append(currentAngleLigament);
        root.append(targetAngleLigament);
    }

    public void updateCurrentMechanismAngle(Rotation2d newAngle) {
        currentAngleLigament.setAngle(newAngle);
        Logger.recordOutput("armMechanism2d", armMechanism);
    }

    public void updateTargetMechanismAngle(Rotation2d newTargetAngle) {
        targetAngleLigament.setAngle(newTargetAngle);
        Logger.recordOutput("armMechanism2d", armMechanism);
    }

    public Mechanism2d getMechanism() {
        return armMechanism;
    }
}