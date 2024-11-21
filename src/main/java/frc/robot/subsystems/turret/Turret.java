package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.subsystems.turret.TurretConstants.*;

public class Turret extends GenericSubsystem {
    public Command autoAimTurret() {
        return Commands.run(() -> setTargetPosition(autoAim()), this);//todo: fix weird spacing. V
    }

    public Command spinTurret() {
        return Commands.run(() -> TURRET_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, 4), this);
    } //todo: is this a test method? if so name it accordingly. V

    /**
     * @param position in rotations
     */
    public Command setTurretToPosition(double position) { //todo: Don't use numbers in function names V
        return Commands.run(() -> setTargetPosition(position), this);  //todo: weird spacing V
    }

    public Command stop() {
        return Commands.runOnce(TURRET_MOTOR::stopMotor, this); //todo: requirement where? V
    }

    public Rotation2d getCurrentTurretPosition() {
        return Rotation2d.fromRotations(TURRET_MOTOR.getSystemPosition());
    }

    @Override
    public void periodic() {
        TURRET_POSE_3D.updateComponent(TURRET_PITCH, getCurrentTurretPosition());
        //todo: instead of initializing a new rotation2d object every loop, pass a predefined rotation2d object with the value of 0 V
    }

    /**
     * @Units in rotations
     */
    private void setTargetPosition(double targetPosition) {
        setAndOptimizeOutput(targetPosition, getCurrentTurretPosition().getRotations());
    }

    private double autoAim() {
        final Translation2d robotPose = POSE_ESTIMATOR.getCurrentPose().getTranslation();
        final Translation2d distanceToHub = HUB_POSITION.minus(robotPose);

        return Units.radiansToRotations(Math.atan2(distanceToHub.getY(), distanceToHub.getX()));
        //todo: cool impl. however, you should use the conventional Translation2d to get the angle between two poses like so:
        //  diff = translation1.minus(translation2
        // return atan2(diff#getY, diff#getX);
    }

    /**
     * @Units in rotations
     */
    private void setAndOptimizeOutput(double targetPosition, double currentPosition) {
        //todo: horrible name for a function. describe what it is DOING, not how. V
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
