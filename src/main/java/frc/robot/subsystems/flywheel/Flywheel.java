package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.flywheel.FlywheelConstants.FLYWHEEL_MOTOR;

public class Flywheel extends GenericSubsystem {
    public Command setFlywheelVoltage(double voltage) {
        return Commands.run(() -> setVoltage(voltage), this);
    }

    public Command stop() {
        return Commands.runOnce(FLYWHEEL_MOTOR::stopMotor, this);
    }

    public Rotation2d getCurrentVelocity() {
        return Rotation2d.fromRotations(FLYWHEEL_MOTOR.getSystemVelocity());
    }

    private void setVoltage(double voltage) {
        FLYWHEEL_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }
}
