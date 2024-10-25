package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.flywheel.FlywheelConstants.FLYWHEEL_MOTOR;

public class Flywheel {
    public Command setFlywheelVelocity() {
        return Commands.run(this::setVoltage);
    }

    public Command stop() {
        return Commands.runOnce(FLYWHEEL_MOTOR::stopMotor);
    }

    private void setVoltage() {
        FLYWHEEL_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, 10);
    }
}
