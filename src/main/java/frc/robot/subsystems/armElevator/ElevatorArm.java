package frc.robot.subsystems.armElevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.armElevator.ElevatorArmConstants.*;

public class ElevatorArm extends GenericSubsystem {
    /**
     * @param position in rotations
     */
    public Command setElevatorArmPosition(double position) {
        return Commands.run(() -> ELEVATOR_ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, position), this);
    }

    /**
     * @param position in rotations
     */
    public Command setArmElevatorPosition(double position) {
        return Commands.run(() -> ARM_ELEVATOR_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, position), this);
    }

    public Command stopElevatorArm() {
        return Commands.runOnce(ELEVATOR_ARM_MOTOR::stopMotor, this);
    }

    public Command stopArmElevator() {
        return Commands.runOnce(ARM_ELEVATOR_MOTOR::stopMotor, this);
    }

    public double getCurrentElevatorArmPosition() {
        return ELEVATOR_ARM_MOTOR.getSystemPosition();
    }

    public double getTargetElevatorArmPosition() {
        return ELEVATOR_ARM_MOTOR.getClosedLoopTarget();
    }

    public double getCurrentArmElevatorPosition() {
        return ARM_ELEVATOR_MOTOR.getSystemPosition();
    }

    public double getTargetArmElevatorPosition() {
        return ARM_ELEVATOR_MOTOR.getClosedLoopTarget();
    }

    @Override
    public void periodic() {
        elevatorArmMechanism.updateCurrentPosition(getTargetElevatorArmPosition() * 10);
        elevatorArmMechanism.updateTargetPosition(getCurrentElevatorArmPosition() * 10);
        elevatorArmMechanism.updateCurrentAngle(Rotation2d.fromRotations(getTargetArmElevatorPosition()));
        elevatorArmMechanism.updateTargetAngle(Rotation2d.fromRotations(getCurrentArmElevatorPosition()));
    }
}
