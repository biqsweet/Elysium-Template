package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.arm.ArmConstants.ARM_MOTOR;

public class Arm extends GenericSubsystem {
    public Command setIntakeVelocity() {
        return Commands.run(this::setVelocity, this);
    }

    private void setVelocity() {
        ARM_MOTOR.setOutput(MotorProperties.ControlMode.VELOCITY, 10);
    }
}
