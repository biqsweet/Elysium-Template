package frc.robot.subsystems.doubleJointedArm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.doubleJointedArm.DoubleJointedArmConstants.*;

public class DoubleJointedArm extends GenericSubsystem {
    /**
     * @param position in rotations
     */
    public Command setShoulderPosition(double position) {
        return Commands.run(() -> SHOULDER_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, position), this);
    }

    /**
     * @param position in rotations
     */
    public Command setElbowPosition(double position) {
        return Commands.run(() -> ELBOW_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, position), this);
    }

    public Command stopShoulder() {
        return Commands.runOnce(SHOULDER_MOTOR::stopMotor, this);
    }

    public Command stopElbow() {
        return Commands.runOnce(SHOULDER_MOTOR::stopMotor, this);
    }

    public Rotation2d getCurrentShoulderPosition() {
        return Rotation2d.fromRotations(SHOULDER_MOTOR.getSystemPosition());
    }

    public Rotation2d getTargetShoulderPosition() {
        return Rotation2d.fromRotations(SHOULDER_MOTOR.getClosedLoopTarget());
    }

    public Rotation2d getCurrentElbowPosition() {
        return Rotation2d.fromRotations(ELBOW_MOTOR.getSystemPosition());
    }

    public Rotation2d getTargetElbowPosition() {
        return Rotation2d.fromRotations(ELBOW_MOTOR.getClosedLoopTarget());
    }

    @Override
    public void periodic() {
        doubleJointedArmMechanism2d.updateTargetAngle(getTargetShoulderPosition(), getTargetElbowPosition());
        doubleJointedArmMechanism2d.updateCurrentAngle(getCurrentShoulderPosition(), getCurrentElbowPosition());
    }
}
