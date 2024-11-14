package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.intake.IntakeArmConstants.*;


public class IntakeArm extends GenericSubsystem {
    public Command engageIntakeArm() {
        return Commands.run(() -> setTargetPosition(true), this);
    }

    public Command disengageIntakeArm() {
        return Commands.run(() -> setTargetPosition(false), this);
    }

    public Command stop() {
        return Commands.runOnce(INTAKE_ARM_MOTOR::stopMotor);
    @Override
    public void periodic() {
        INTAKE_ARM_POSE_3D.updateComponent(getCurrentIntakeArmPosition(), INTAKE_ARM_YAW);
    }

    private Rotation2d getCurrentIntakeArmPosition() {
        return Rotation2d.fromRotations(INTAKE_ARM_MOTOR.getSystemPosition());
    }

    private void setTargetPosition(boolean engaged) {
        INTAKE_ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, engaged ? 0 : 0.25);
    }
}
