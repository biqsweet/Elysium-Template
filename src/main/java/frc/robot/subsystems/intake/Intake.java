package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;


import static frc.robot.subsystems.intake.IntakeConstants.*;

public class Intake extends GenericSubsystem {
    public Command setIntakeVelocity() {
        return Commands.run(this::setVelocity, this);
    }

    public double getVelocity(){
        return INTAKE_MOTOR.getMotorVelocity();
    }

    private void setVelocity() {
        INTAKE_MOTOR.setOutput(MotorProperties.ControlMode.VELOCITY, 10);
    }
}
