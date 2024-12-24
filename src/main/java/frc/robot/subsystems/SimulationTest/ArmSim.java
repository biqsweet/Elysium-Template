package frc.robot.subsystems.SimulationTest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.SimulationTest.ArmSimConstants.ARM_MOTOR;
import static frc.robot.subsystems.SimulationTest.ArmSimConstants.armMechanism;

public class ArmSim extends GenericSubsystem {
    /**
     * @param position in rotations
     */
    public Command setArmPosition(double position) {
        return Commands.run(() -> ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, position), this);
    }

    public Command stopArm() {
        return Commands.runOnce(ARM_MOTOR::stopMotor, this);
    }

    public Rotation2d getCurrentPosition() {
        return Rotation2d.fromRotations(ARM_MOTOR.getSystemPosition());
    }

    public Rotation2d getTargetPosition() {
        return Rotation2d.fromRotations(ARM_MOTOR.getClosedLoopTarget());
    }

    @Override
    public void periodic() {
        armMechanism.updateTargetAngle(getTargetPosition());
        armMechanism.updateCurrentAngle(getCurrentPosition());
    }
}
