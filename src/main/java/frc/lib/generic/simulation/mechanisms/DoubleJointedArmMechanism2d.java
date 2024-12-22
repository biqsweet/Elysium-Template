package frc.lib.generic.simulation.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import org.littletonrobotics.junction.Logger;

import static frc.lib.generic.simulation.mechanisms.MechanismConstants.*;
import static frc.lib.generic.simulation.mechanisms.MechanismUtilities.getRoot;

public class DoubleJointedArmMechanism2d {
    private final String name;
    private final Mechanism2d doubleJointedArmMechanism;
    private final MechanismRoot2d root;
    private MechanismLigament2d
            currentShoulderLigament,
            targetShoulderLigament,
            currentElbowLigament,
            targetElbowLigament;

    public DoubleJointedArmMechanism2d(String name, double shoulderLength, double elbowLength, Rotation2d minimumShoulderAngle, Rotation2d minimumElbowAngle) {
        this.name = name;
        this.doubleJointedArmMechanism = new Mechanism2d(DEFAULT_CANVAS_WIDTH, DEFAULT_CANVAS_HEIGHT);
        this.root = getRoot("DoubleJointedArmRoot", doubleJointedArmMechanism);

        createCurrentLigaments(shoulderLength, elbowLength, minimumShoulderAngle, minimumElbowAngle);
        createTargetLigaments(shoulderLength, elbowLength, minimumShoulderAngle, minimumElbowAngle);
    }

    public void updateCurrentAngle(Rotation2d shoulderAngle, Rotation2d elbowAngle) {
        currentShoulderLigament.setAngle(shoulderAngle);
        currentElbowLigament.setAngle(elbowAngle.minus(shoulderAngle));
        Logger.recordOutput(name, doubleJointedArmMechanism);
    }

    public void updateTargetAngle(Rotation2d targetShoulderAngle, Rotation2d targetElbowAngle) {
        targetShoulderLigament.setAngle(targetShoulderAngle);
        targetElbowLigament.setAngle(targetElbowAngle.minus(targetShoulderAngle));
        Logger.recordOutput(name, doubleJointedArmMechanism);
    }

    public Mechanism2d getMechanism() {
        return doubleJointedArmMechanism;
    }

    private void createCurrentLigaments(double shoulderLength, double elbowLength, Rotation2d defaultShoulderAngle, Rotation2d defaultElbowAngle) {
        currentShoulderLigament = new MechanismLigament2d("currentShoulderLigament", shoulderLength, defaultShoulderAngle.getDegrees(), DEFAULT_LINE_WIDTH, BLUE);
        currentElbowLigament = new MechanismLigament2d("currentElbowLigament", elbowLength, defaultElbowAngle.getDegrees(), DEFAULT_LINE_WIDTH, DARK_BLUE);

        currentShoulderLigament.append(currentElbowLigament);
        root.append(currentShoulderLigament);

    }

    private void createTargetLigaments(double shoulderLength, double elbowLength, Rotation2d defaultShoulderAngle, Rotation2d defaultElbowAngle) {
        targetShoulderLigament = new MechanismLigament2d("targetShoulderLigament", shoulderLength, defaultShoulderAngle.getDegrees(), DEFAULT_LINE_WIDTH, GRAY);
        targetElbowLigament = new MechanismLigament2d("targetElbowLigament", elbowLength, defaultElbowAngle.getDegrees(), DEFAULT_LINE_WIDTH, DARK_GRAY);

        targetShoulderLigament.append(targetElbowLigament);
        root.append(targetShoulderLigament);
    }
}