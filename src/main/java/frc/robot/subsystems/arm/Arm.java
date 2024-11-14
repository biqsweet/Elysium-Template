package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.RobotContainer.TURRET;
import static frc.robot.subsystems.arm.ArmConstants.ARM_MOTOR;
import static frc.robot.subsystems.arm.ArmConstants.ARM_POSE_3D;

public class Arm extends GenericSubsystem {
    /**
     * @param position - in rotations
     */
    public Command setArmPosition(double position) {
        return Commands.run(() -> ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, position), this);
    }


    public Command stop() {
        return Commands.runOnce(ARM_MOTOR::stopMotor, this);  //todo:  requires what? V
    }

    @Override
    public void periodic() {
        ARM_POSE_3D.updateComponent(getCurrentArmPosition(), TURRET.getCurrentTurretPosition());
    }

    public Rotation2d getCurrentArmPosition() {
        return Rotation2d.fromRotations(ARM_MOTOR.getSystemPosition());
    }

    private void setTargetPosition(boolean position) {
        ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, position ? 0.15 : 0);
    }
}
