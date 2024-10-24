package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.intake.IntakeArmConstants.INTAKE_ARM_MOTOR;


public class IntakeArm extends GenericSubsystem {
    public Command engageIntakeArm() {
        return Commands.run(() -> setTargetPosition(true));
    }
    public Command disengageIntakeArm() {
        return Commands.run(() -> setTargetPosition(false));
    }

    private void setTargetPosition(boolean engaged) {
        INTAKE_ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, engaged ? 0 : 45);
    }
}
