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

    public DoubleJointedArmMechanism2d(String name, double shoulderLength, double elbowLength, Rotation2d minimumShoulderAngle, Rotation2d maximumShoulderAngle, Rotation2d minimumElbowAngle, Rotation2d maximumElbowAngle) {
        this.name = name;
        this.doubleJointedArmMechanism = new Mechanism2d(DEFAULT_CANVAS_WIDTH, DEFAULT_CANVAS_HEIGHT);
        this.root = getRoot("DoubleJointedArmRoot", doubleJointedArmMechanism);

        createCurrent(shoulderLength, elbowLength, minimumShoulderAngle, maximumShoulderAngle, minimumElbowAngle, maximumElbowAngle);
        createTarget(shoulderLength, elbowLength, minimumShoulderAngle, maximumShoulderAngle, minimumElbowAngle, maximumElbowAngle);
    }

    public void updateCurrentMechanismAngle(Rotation2d newShoulderAngle, Rotation2d newElbowAngle) {
        currentShoulderLigament.setAngle(newShoulderAngle);
        currentElbowLigament.setAngle(newElbowAngle.minus(newShoulderAngle));
        Logger.recordOutput(name, doubleJointedArmMechanism);
    }

    public void updateTargetMechanismAngle(Rotation2d newTargetShoulderAngle, Rotation2d newTargetElbowAngle) {
        targetShoulderLigament.setAngle(newTargetShoulderAngle);
        targetElbowLigament.setAngle(newTargetElbowAngle.minus(newTargetShoulderAngle));
        Logger.recordOutput(name, doubleJointedArmMechanism);
    }

    public Mechanism2d getMechanism() {
        return doubleJointedArmMechanism;
    }

    private void createCurrent(double shoulderLength, double elbowLength, Rotation2d minimumShoulderAngle, Rotation2d maximumShoulderAngle, Rotation2d minimumElbowAngle, Rotation2d maximumElbowAngle) {
        currentShoulderLigament = new MechanismLigament2d("currentShoulderLigament", shoulderLength, minimumShoulderAngle.getDegrees(), DEFAULT_LINE_WIDTH, BLUE);
        currentElbowLigament = new MechanismLigament2d("currentElbowLigament", elbowLength, minimumElbowAngle.getDegrees(), DEFAULT_LINE_WIDTH, DARK_BLUE);

        currentShoulderLigament.append(currentElbowLigament);
        root.append(currentShoulderLigament);

    }

    private void createTarget(double shoulderLength, double elbowLength, Rotation2d minimumShoulderAngle, Rotation2d maximumShoulderAngle, Rotation2d minimumElbowAngle, Rotation2d maximumElbowAngle) {
        targetShoulderLigament = new MechanismLigament2d("targetShoulderLigament", shoulderLength, minimumShoulderAngle.getDegrees(), DEFAULT_LINE_WIDTH, GRAY);
        targetElbowLigament = new MechanismLigament2d("targetElbowLigament", elbowLength, minimumElbowAngle.getDegrees(), DEFAULT_LINE_WIDTH, DARK_GRAY);

        targetShoulderLigament.append(targetElbowLigament);
        root.append(targetShoulderLigament);
    }
}
