package frc.robot.subsystems.SimulationTest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.SimulationTest.FlywheelConstants.FLYWHEEL_MOTOR;
import static frc.robot.subsystems.SimulationTest.FlywheelConstants.flywheelMechanism;

public class Flywheel extends GenericSubsystem {
    public Command setFlywheelVoltage(double voltage) {
        return Commands.run(() -> FLYWHEEL_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage), this);
    }

    public Command stop() {
        return Commands.runOnce(FLYWHEEL_MOTOR::stopMotor, this);
    }

    public double getCurrentVoltage() {
        return FLYWHEEL_MOTOR.getVoltage();
    }

    public Rotation2d getCurrentVelocity() {
        return Rotation2d.fromRotations(FLYWHEEL_MOTOR.getSystemVelocity());
    }

    public double getTargetVoltage() {
        return FLYWHEEL_MOTOR.getClosedLoopTarget();
    }

    @Override
    public void periodic() {
        flywheelMechanism.updateCurrentSpeed(getCurrentVoltage());
        flywheelMechanism.updateTargetSpeed(getTargetVoltage());
    }
}
