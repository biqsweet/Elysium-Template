package frc.lib.generic.simulation.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import org.littletonrobotics.junction.Logger;

import static frc.lib.generic.simulation.mechanisms.MechanismConstants.*;
import static frc.lib.generic.simulation.mechanisms.MechanismUtilities.getRoot;

public class SpeedMechanism2d {
    private final String name;
    private final Mechanism2d speedMechanism;
    private final MechanismRoot2d root;
    private MechanismLigament2d
            currentSpeedLigament,
            targetSpeedLigament,
            currentArrowTopLigaments,
            currentArrowBottomLigaments,
            targetArrowTopLigaments,
            targetArrowBottomLigaments;

    public SpeedMechanism2d(String name) {
        this.name = name;
        this.speedMechanism = new Mechanism2d(DEFAULT_CANVAS_WIDTH, DEFAULT_CANVAS_HEIGHT);
        this.root = getRoot("speedMechanismRoot", speedMechanism);

        createCurrentLigaments();
        createTargetLigaments();
    }

    public void updateCurrentSpeed(double speed) {
        if (speed < 0) {
            currentSpeedLigament.setColor(RED);
            currentArrowTopLigaments.setColor(RED);
            currentArrowBottomLigaments.setColor(RED);

            currentArrowTopLigaments.setAngle(ARROW_TOP_ANGLE_INVERSE);
            currentArrowBottomLigaments.setAngle(ARROW_BOTTOM_ANGLE_INVERSE);
        } else {
            currentSpeedLigament.setColor(GREEN);
            currentArrowTopLigaments.setColor(GREEN);
            currentArrowBottomLigaments.setColor(GREEN);

            currentArrowTopLigaments.setAngle(ARROW_TOP_ANGLE);
            currentArrowBottomLigaments.setAngle(ARROW_BOTTOM_ANGLE);
        }

        currentSpeedLigament.setLength(speed);
        Logger.recordOutput(name, speedMechanism);
    }

    public void updateTargetSpeed(double targetSpeed) {
        if (targetSpeed < 0) {
            targetArrowTopLigaments.setAngle(ARROW_TOP_ANGLE_INVERSE);
            targetArrowBottomLigaments.setAngle(ARROW_BOTTOM_ANGLE_INVERSE);
        } else {
            targetArrowTopLigaments.setAngle(ARROW_TOP_ANGLE);
            targetArrowBottomLigaments.setAngle(ARROW_BOTTOM_ANGLE);
        }

        targetSpeedLigament.setLength(targetSpeed);
        Logger.recordOutput(name, speedMechanism);
    }

    public Mechanism2d getMechanism() {
        return speedMechanism;
    }

    private void createCurrentLigaments() {
        currentSpeedLigament = new MechanismLigament2d("currentSpeed", 5, 0, DEFAULT_LINE_WIDTH, GREEN);
        currentArrowTopLigaments = new MechanismLigament2d("currentArrowTip1", 1, ARROW_TOP_ANGLE, DEFAULT_LINE_WIDTH, GREEN);
        currentArrowBottomLigaments = new MechanismLigament2d("currentArrowTip2", 1, ARROW_BOTTOM_ANGLE, DEFAULT_LINE_WIDTH, GREEN);

        currentSpeedLigament.append(currentArrowTopLigaments);
        currentSpeedLigament.append(currentArrowBottomLigaments);
        root.append(currentSpeedLigament);
    }

    private void createTargetLigaments() {
        targetSpeedLigament = new MechanismLigament2d("targetSpeed", 5, 0, DEFAULT_LINE_WIDTH, GRAY);
        targetArrowTopLigaments = new MechanismLigament2d("targetArrowTip1", 1, ARROW_TOP_ANGLE, DEFAULT_LINE_WIDTH, GRAY);
        targetArrowBottomLigaments = new MechanismLigament2d("targetArrowTip2", 1, ARROW_BOTTOM_ANGLE, DEFAULT_LINE_WIDTH, GRAY);

        targetSpeedLigament.append(targetArrowTopLigaments);
        targetSpeedLigament.append(targetArrowBottomLigaments);
        root.append(targetSpeedLigament);
    }
}