package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.subsystems.turret.TurretConstants.*;

public class Turret extends GenericSubsystem {
    public Command autoAimTurret() {
        double robotX = POSE_ESTIMATOR.getCurrentPose().getX();
        double robotY = POSE_ESTIMATOR.getCurrentPose().getY();

        Rotation2d angleToHub = Rotation2d.fromDegrees(Math.atan(HUB_Y - robotY / HUB_X - robotX));

        return Commands.run(() -> setTargetPosition(angleToHub), this);
    }

    public Command stop() {
        return Commands.runOnce(TURRET_MOTOR::stopMotor);
    }

    public Rotation2d getCurrentTurretPosition() {
        return Rotation2d.fromRotations(TURRET_MOTOR.getSystemPosition());
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

        if (targetPosition.getDegrees() > MAX_ANGLE.getDegrees() ||
                targetPosition.getDegrees() < MIN_ANGLE.getDegrees() ||
                targetPosition.getDegrees() > currentPosition.getDegrees() + 180) {
            return -K_P * (360 - error);
        } else {
            return K_P * error;
        }
    }


}
