package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.intake.IntakeArmConstants.*;


public class IntakeArm extends GenericSubsystem {
    /**
     * @param position - in rotations
     */
    public Command setIntakeArmPosition(double position) {
        return Commands.run(() -> INTAKE_ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, position), this);
    }

    public Command stop() {
        return Commands.runOnce(INTAKE_ARM_MOTOR::stopMotor, this);
    } //todo req V

    @Override
    public void periodic() {
        INTAKE_ARM_POSE_3D.updateComponent(getCurrentIntakeArmPosition(), INTAKE_ARM_YAW);
    }

    private Rotation2d getCurrentIntakeArmPosition() {
        return Rotation2d.fromRotations(INTAKE_ARM_MOTOR.getSystemPosition());
    }

    /**
     * @param position - in rotations
     */
    private void setTargetPosition(double position) {
        INTAKE_ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, position); //todo get a double not a boolean V
    }
}
