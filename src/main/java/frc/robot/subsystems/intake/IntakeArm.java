package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.intake.IntakeArmConstants.INTAKE_ARM_MOTOR;


public class IntakeArm extends GenericSubsystem {
    public Command setIntakeArmPosition(Rotation2d targetPosition) {
        return Commands.run(() -> setTargetPosition(targetPosition));
    }

    private void setTargetPosition(Rotation2d targetPosition) {
        INTAKE_ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, targetPosition.getRotations());
    }
}
