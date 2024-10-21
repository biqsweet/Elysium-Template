package frc.robot.subsystems.verticalconveyor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.verticalconveyor.VerticalConveyorConstants.CONVEYOR_MOTOR;

public class VerticalConveyor extends GenericSubsystem {
    public Command setConveyorVelocity() {
        return Commands.run(this::setVelocity, this);
    }

    private void setVelocity() {
        CONVEYOR_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, 4);
    }
}
