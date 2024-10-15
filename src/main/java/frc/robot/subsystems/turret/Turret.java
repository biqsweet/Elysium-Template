package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.turret.TurretConstants.TURRET_MOTOR;

public class Turret extends GenericSubsystem {
    public Command setTurretPosition(Rotation2d targetPosition) {
        return Commands.run(() -> setTargetPosition(targetPosition));
    }

    private void setTargetPosition(Rotation2d targetPosition) {
        TURRET_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, targetPosition.getRotations());
    }
}
