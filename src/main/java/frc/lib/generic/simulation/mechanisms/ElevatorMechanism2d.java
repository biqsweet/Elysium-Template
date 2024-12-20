package frc.lib.generic.simulation.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import org.littletonrobotics.junction.Logger;

import static frc.lib.generic.simulation.mechanisms.MechanismConstants.*;
import static frc.lib.generic.simulation.mechanisms.MechanismUtilities.getRoot;


public class ElevatorMechanism2d {
    private final String name;
    private final Mechanism2d elevatorMechanism;
    private final MechanismRoot2d
            root,
            targetRoot;
    private MechanismLigament2d
            currentRightLigament,
            currentLeftLigament,
            targetRightLigament,
            targetLeftLigament;

    public ElevatorMechanism2d(String name, double elevatorLength) {
        this.name = name;
        this.elevatorMechanism = new Mechanism2d(DEFAULT_CANVAS_WIDTH, 20);

        this.root = getRoot("elevatorRoot", elevatorMechanism);
        this.targetRoot = getRoot("elevatorTargetRoot", elevatorMechanism);

        createCurrent(elevatorLength);
        createTarget(elevatorLength);
    }

    public void updateCurrentMechanismPosition(double newPosY) {
        root.setPosition(DEFAULT_ROOT_X, DEFAULT_ROOT_Y + newPosY);
        Logger.recordOutput(name, elevatorMechanism);
    }

    public void updateTargetMechanismPosition(double newTargetPosY) {
        targetRoot.setPosition(DEFAULT_ROOT_X, DEFAULT_ROOT_Y + newTargetPosY);
        Logger.recordOutput(name, elevatorMechanism);
    }

    public Mechanism2d getMechanism() {
        return elevatorMechanism;
    }

    public void createCurrent(double elevatorLength) {
        this.currentRightLigament = new MechanismLigament2d("elevatorRightLigament", elevatorLength, 0, DEFAULT_LINE_WIDTH, RED);
        this.currentLeftLigament = new MechanismLigament2d("elevatorLeftLigament", elevatorLength, 180, DEFAULT_LINE_WIDTH, RED);

        root.append(currentRightLigament);
        root.append(currentLeftLigament);
    }

    public void createTarget(double elevatorLength) {
        this.targetRightLigament = new MechanismLigament2d("targetRightLigament", elevatorLength, 0, DEFAULT_LINE_WIDTH, BLUE);
        this.targetLeftLigament = new MechanismLigament2d("targetLeftLigament", elevatorLength, 180, DEFAULT_LINE_WIDTH, BLUE);

        targetRoot.append(targetRightLigament);
        targetRoot.append(targetLeftLigament);
    }

    public void createOutline() {
        MechanismRoot2d outlineRoot = elevatorMechanism.getRoot("outlineRoot", 1, 1);

        MechanismLigament2d
                outlineTop = new MechanismLigament2d("outlineTop", 8, -90, DEFAULT_LINE_WIDTH, GRAY),
                outlineLeft = new MechanismLigament2d("outlineLeft", 18, 90, DEFAULT_LINE_WIDTH, GRAY),
                outlineRight = new MechanismLigament2d("outlineRight", 18, -90, DEFAULT_LINE_WIDTH, GRAY);

        outlineTop.append(outlineRight);
        outlineRoot.append(outlineLeft);
        outlineLeft.append(outlineTop);
    }
}
