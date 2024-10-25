package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.turret.TurretConstants.*;

public class Turret extends GenericSubsystem {
    public Command setTurretPosition(Rotation2d targetPosition) {
        return Commands.run(() -> setTargetPosition(targetPosition));
    }

    public Command stop() {
        return Commands.runOnce(TURRET_MOTOR::stopMotor);
    }

    public void getCurrentPosition() {
        TURRET_MOTOR.getSystemPosition();
    }

    private void setTargetPosition(Rotation2d targetPosition) {
        pController(targetPosition, Rotation2d.fromDegrees(TURRET_MOTOR.getSystemPosition()));
    }

    private void pController(Rotation2d targetPosition, Rotation2d currentPosition) {
        double output = optimize(targetPosition, currentPosition);
        TURRET_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, output);
    }

    private double optimize(Rotation2d targetPosition, Rotation2d currentPosition) {
        double error = targetPosition.getDegrees() - currentPosition.getDegrees();

        if (targetPosition.getDegrees() > MAX_ANGLE.getDegrees() || targetPosition.getDegrees() < MIN_ANGLE.getDegrees() || targetPosition.getDegrees() > currentPosition.getDegrees() + 180) {
            return -K_P * (360 - error);
        } else {
            return K_P * error;
        }
    }


}
