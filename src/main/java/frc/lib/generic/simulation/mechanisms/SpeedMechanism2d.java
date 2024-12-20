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
            currentArrowTip1,
            currentArrowTip2,
            targetArrowTip1,
            targetArrowTip2;

    public SpeedMechanism2d(String name) {
        this.name = name;
        this.speedMechanism = new Mechanism2d(DEFAULT_CANVAS_WIDTH, DEFAULT_CANVAS_HEIGHT);
        this.root = getRoot("speedMechanismRoot", speedMechanism);

        createCurrent();
        createTarget();
    }

    public void updateCurrentMechanismSpeed(double newSpeed) {
        if (newSpeed < 0) {
            currentSpeedLigament.setColor(RED);
            currentArrowTip1.setColor(RED);
            currentArrowTip2.setColor(RED);

            currentArrowTip1.setAngle(DEFAULT_ARROW_TIP1_INVERSE);
            currentArrowTip2.setAngle(DEFAULT_ARROW_TIP2_INVERSE);
        } else {
            currentSpeedLigament.setColor(GREEN);
            currentArrowTip1.setColor(GREEN);
            currentArrowTip2.setColor(GREEN);

            currentArrowTip1.setAngle(DEFAULT_ARROW_TIP1);
            currentArrowTip2.setAngle(DEFAULT_ARROW_TIP1);
        }

        currentSpeedLigament.setLength(newSpeed);
        Logger.recordOutput(name, speedMechanism);
    }

    public void updateTargetMechanismSpeed(double newTargetSpeed) {
        if (newTargetSpeed < 0) {
            targetArrowTip1.setAngle(DEFAULT_ARROW_TIP1_INVERSE);
            targetArrowTip2.setAngle(DEFAULT_ARROW_TIP2_INVERSE);
        } else {
            targetArrowTip1.setAngle(DEFAULT_ARROW_TIP1);
            targetArrowTip2.setAngle(DEFAULT_ARROW_TIP2);
        }

        targetSpeedLigament.setLength(newTargetSpeed);
        Logger.recordOutput(name, speedMechanism);
    }

    public Mechanism2d getMechanism() {
        return speedMechanism;
    }

    private void createCurrent() {
        currentSpeedLigament = new MechanismLigament2d("currentSpeed", 5, 0, DEFAULT_LINE_WIDTH, GREEN);
        currentArrowTip1 = new MechanismLigament2d("currentArrowTip1", 1, DEFAULT_ARROW_TIP1, DEFAULT_LINE_WIDTH, GREEN);
        currentArrowTip2 = new MechanismLigament2d("currentArrowTip2", 1, DEFAULT_ARROW_TIP2, DEFAULT_LINE_WIDTH, GREEN);

        currentSpeedLigament.append(currentArrowTip1);
        currentSpeedLigament.append(currentArrowTip2);
        root.append(currentSpeedLigament);
    }

    private void createTarget() {
        targetSpeedLigament = new MechanismLigament2d("targetSpeed", 5, 0, DEFAULT_LINE_WIDTH, GRAY);
        targetArrowTip1 = new MechanismLigament2d("targetArrowTip1", 1, DEFAULT_ARROW_TIP1, DEFAULT_LINE_WIDTH, GRAY);
        targetArrowTip2 = new MechanismLigament2d("targetArrowTip2", 1, DEFAULT_ARROW_TIP2, DEFAULT_LINE_WIDTH, GRAY);

        targetSpeedLigament.append(targetArrowTip1);
        targetSpeedLigament.append(targetArrowTip2);
        root.append(targetSpeedLigament);
    }
}