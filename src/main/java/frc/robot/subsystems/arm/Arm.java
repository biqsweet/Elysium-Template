package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.GlobalConstants.HUB_POSITION;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.TURRET;
import static frc.robot.subsystems.arm.ArmConstants.ARM_MOTOR;
import static frc.robot.subsystems.arm.ArmConstants.ARM_POSE_3D;

public class Arm extends GenericSubsystem {
    /**
     * @param position in rotations
     */
    public Command setArmPosition(double position) {
        return Commands.run(() -> ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, position), this);
    }

    public Command autoAimArm() {
        return Commands.run(() -> ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, autoAim()), this);
    }

    public Command spinArm() {
        return Commands.run(() -> ARM_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, 5), this);
    }

    public Command stop() {
        return Commands.runOnce(ARM_MOTOR::stopMotor, this);
    }

    @Override
    public void periodic() {
        ARM_POSE_3D.updateComponent(getCurrentArmPosition().plus(Rotation2d.fromRotations(0.15)), TURRET.getCurrentTurretPosition(), new Translation3d(0, 0, 0));
    }

    public Rotation2d getCurrentArmPosition() {
        return Rotation2d.fromRotations(ARM_MOTOR.getSystemPosition());
    }

    public Rotation2d getTargetArmPosition() {
        return Rotation2d.fromRotations(ARM_MOTOR.getClosedLoopTarget());
    }

    private double autoAim() {
        final Translation2d robotPose = POSE_ESTIMATOR.getCurrentPose().getTranslation();
        final Translation2d distanceToHub = HUB_POSITION.toTranslation2d().minus(robotPose);

        return Units.radiansToRotations(
                Math.atan2(
                        distanceToHub.getNorm(), HUB_POSITION.getZ() - 1
                ) + Units.degreesToRadians(10)
        );
    }
}
