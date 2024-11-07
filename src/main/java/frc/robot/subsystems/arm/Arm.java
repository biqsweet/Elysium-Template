package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.arm.ArmConstants.ARM_MOTOR;

public class Arm extends GenericSubsystem {
    public Command setArmPosition45() {
        return Commands.run(() -> setTargetPosition(true), this);
    }

    public Command setArmPosition10() {
        return Commands.run(() -> setTargetPosition(false), this);
    }

    public Command stop() {
        return Commands.runOnce(ARM_MOTOR::stopMotor);
    }

    public void getCurrentPosition() {
        ARM_MOTOR.getSystemPosition();
    private Rotation2d getCurrentPosition() {
    public Rotation2d getCurrentArmPosition() {
        return Rotation2d.fromRotations(ARM_MOTOR.getSystemPosition());
    }

    private void setTargetPosition(boolean position) {
        ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, position ? 0.25 : 0);
    }
}
