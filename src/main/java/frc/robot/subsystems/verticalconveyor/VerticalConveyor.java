package frc.robot.subsystems.verticalconveyor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.verticalconveyor.VerticalConveyorConstants.CONVEYOR_MOTOR;

public class VerticalConveyor extends GenericSubsystem {
    public Command setConveyorVoltage() {
        return Commands.run(this::setVoltage, this);
    }

    public Command stop() {
        return Commands.runOnce(CONVEYOR_MOTOR::stopMotor);
    }

    private Rotation2d getCurrentPosition() {
        return Rotation2d.fromRotations(CONVEYOR_MOTOR.getSystemPosition());
    }

    private void setVoltage() {
        CONVEYOR_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, 4);
    }
}
