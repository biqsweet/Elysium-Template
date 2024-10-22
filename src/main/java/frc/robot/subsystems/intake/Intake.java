package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_MOTOR;

public class Intake extends GenericSubsystem {
    public Command setIntakeVelocity(boolean isInverted) {
        return Commands.run(() -> setVelocity(isInverted));
    }

    private void setVelocity(boolean isInverted) {
        INTAKE_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, isInverted ? -4 : 4);
    }
}
