package frc.lib.generic.simulation.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import org.littletonrobotics.junction.Logger;

import static frc.lib.generic.simulation.mechanisms.MechanismConstants.*;
import static frc.lib.generic.simulation.mechanisms.MechanismUtilities.getRoot;


public class ElevatorArmMechanism2d {
    private final String name;
    private final Mechanism2d armElevatorMechanism;
    private final MechanismRoot2d
            root,
            targetRoot;
    private MechanismLigament2d
            currentLigament,
            targetLigament;

    public ElevatorArmMechanism2d(String name, double elevatorLength) {
        this.name = name;
        this.armElevatorMechanism = new Mechanism2d(DEFAULT_CANVAS_WIDTH, 20);

        this.root = getRoot("armElevatorRoot", armElevatorMechanism);
        this.targetRoot = getRoot("armElevatorTargetRoot", armElevatorMechanism);

        createCurrentLigaments(elevatorLength);
        createTargetLigaments(elevatorLength);
        createOutline();
    }

    public void updateCurrentPosition(double PosY) {
        root.setPosition(DEFAULT_ROOT_X, DEFAULT_ROOT_Y + PosY);
        Logger.recordOutput(name, armElevatorMechanism);
    }

    public void updateTargetPosition(double TargetPosY) {
        targetRoot.setPosition(DEFAULT_ROOT_X, DEFAULT_ROOT_Y + TargetPosY);
        Logger.recordOutput(name, armElevatorMechanism);
    }

    public void updateCurrentAngle(Rotation2d currentAngle) {
        currentLigament.setAngle(currentAngle);
        Logger.recordOutput(name, armElevatorMechanism);
    }

    public void updateTargetAngle(Rotation2d targetAngle) {
        targetLigament.setAngle(targetAngle);
        Logger.recordOutput(name, armElevatorMechanism);
    }

    public Mechanism2d getMechanism() {
        return armElevatorMechanism;
    }

    private void createCurrentLigaments(double elevatorLength) {
        this.currentLigament = new MechanismLigament2d("armElevatorLigament", elevatorLength, 0, DEFAULT_LINE_WIDTH, RED);

        root.append(currentLigament);
    }

    private void createTargetLigaments(double elevatorLength) {
        this.targetLigament = new MechanismLigament2d("targetArmElevatorLigament", elevatorLength, 0, DEFAULT_LINE_WIDTH, BLUE);

        targetRoot.append(targetLigament);
    }

    private void createOutline() {
        MechanismRoot2d outlineRoot = armElevatorMechanism.getRoot("outlineRoot", 1, 1);

        MechanismLigament2d
                outlineTop = new MechanismLigament2d("outlineTop", 8, -90, DEFAULT_LINE_WIDTH, GRAY),
                outlineLeft = new MechanismLigament2d("outlineLeft", 18, 90, DEFAULT_LINE_WIDTH, GRAY),
                outlineRight = new MechanismLigament2d("outlineRight", 18, -90, DEFAULT_LINE_WIDTH, GRAY);

        outlineTop.append(outlineRight);
        outlineRoot.append(outlineLeft);
        outlineLeft.append(outlineTop);
    }
}