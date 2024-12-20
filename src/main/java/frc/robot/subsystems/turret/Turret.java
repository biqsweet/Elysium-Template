package frc.robot.subsystems.turret;

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
import static frc.robot.subsystems.turret.TurretConstants.*;

public class Turret extends GenericSubsystem {
    public Command autoAimTurret() {
        return Commands.run(() -> setTargetPosition(autoAim()), this);
    }

    public Command spinTurret() {
        return Commands.run(() -> TURRET_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, 0.5), this);
    }

    /**
     * @param position in rotations
     */
    public Command setTurretToPosition(double position) {
        return Commands.run(() -> setTargetPosition(position), this);
    }

    public Command stop() {
        return Commands.runOnce(TURRET_MOTOR::stopMotor, this);
    }

    public Rotation2d getCurrentTurretPosition() {
        return Rotation2d.fromRotations(TURRET_MOTOR.getSystemPosition());
    }

    public Rotation2d getTargetTurretPosition() {
        return Rotation2d.fromRotations(TURRET_MOTOR.getClosedLoopTarget());
    }

    @Override
    public void periodic() {
        TURRET_POSE_3D.updateComponent(TURRET_PITCH, getCurrentTurretPosition(), new Translation3d(0, 0, 0));
    }

    /**
     * @Units in rotations
     */
    private void setTargetPosition(double targetPosition) {
        setAndOptimizeOutput(targetPosition, getCurrentTurretPosition().getRotations());
    }

    private double autoAim() {
        final Translation2d robotPose = POSE_ESTIMATOR.getCurrentPose().getTranslation();
        final Translation2d distanceToHub = HUB_POSITION.toTranslation2d().minus(robotPose);

        return Units.radiansToRotations(Math.atan2(distanceToHub.getY(), distanceToHub.getX()));
    }

    /**
     * @Units in rotations
     */
    private void setAndOptimizeOutput(double targetPosition, double currentPosition) {
        TURRET_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, optimize(targetPosition, currentPosition));
    }

    /**
     * @Units in rotations
     */
    private double optimize(double targetPosition, double currentPosition) {
        final double error = targetPosition - currentPosition;

        if (targetPosition > MAX_ANGLE.getRotations() ||
                targetPosition < MIN_ANGLE.getRotations() ||
                targetPosition > currentPosition + 0.5) {
            return -K_P * (1 - error);
        } else {
            return K_P * error;
        }
    }
}
