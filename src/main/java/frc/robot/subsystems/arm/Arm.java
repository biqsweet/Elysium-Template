package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.arm.ArmConstants.ARM_MOTOR;

public class Arm extends GenericSubsystem {
    public Command setArmPosition(Rotation2d targetPosition) {
        return Commands.run(() -> setTargetPosition(targetPosition));
    }

    public Command stop() {
        return Commands.runOnce(ARM_MOTOR::stopMotor);
    }

    private void setTargetPosition(Rotation2d targetPosition) {
        ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, targetPosition.getRotations());
    }
}
