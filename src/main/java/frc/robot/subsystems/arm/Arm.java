package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.lib.math.Conversions.rpsToMps;
import static frc.robot.RobotContainer.*;
import static frc.robot.subsystems.arm.ArmConstants.*;

public class Arm extends GenericSubsystem {
    /**
     * @param position in rotations
     */
    public Command setArmPosition(double position) {
        return Commands.run(() -> ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, position), this);
    }


    public Command stop() {
        return Commands.runOnce(ARM_MOTOR::stopMotor, this);  //todo:  requires what? V
        return Commands.runOnce(ARM_MOTOR::stopMotor, this);
    }

    @Override
    public void periodic() {
        ARM_POSE_3D.updateComponent(getCurrentArmPosition(), TURRET.getCurrentTurretPosition());
    }

    public Rotation2d getCurrentArmPosition() {
        return Rotation2d.fromRotations(ARM_MOTOR.getSystemPosition());
    }

}
