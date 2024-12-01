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
    //todo: Instead of creating two methods to do the same thing, introduce a variable into the function to allow any output the end user desires. V


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


    public Command ballTrajectory() {
        double theta = TURRET.getCurrentTurretPosition().getDegrees();
        double phi = getCurrentArmPosition().getDegrees();

        double F = rpsToMps(FLYWHEEL.getCurrentVelocity().getRotations(), 3) / 60;

        double y = F * Math.sin(phi);
        double x = F * Math.sin(phi) * Math.cos(theta);
        double z = F * Math.sin(phi) * Math.sin(theta);

        Translation2d distanceToHub = HUB_POSITION.minus(POSE_ESTIMATOR.getCurrentPose().getTranslation());
        double acceleration = 9.8 / 1000;
        double time = findTime(acceleration, F, distanceToHub.getNorm());

        return Commands.run(() -> BALL.updateObject(new Translation3d(x, y, z-acceleration)), this);
    }

    /**
     * @param acceleration acceleration
     * @param startingV    starting velocity
     * @param distance     deltaX
     */
    private double findTime(double acceleration, double startingV, double distance) {
        // the startV is a vector, the x speed is constant but the y speed is going down cuz of gravity
        return quadraticFormula(acceleration, startingV, distance);
    }

    /**
     * @return only the positive answer
     */
    private double quadraticFormula(double a, double b, double c) {
        final double discriminant = Math.sqrt(b * b - 4 * a * c);
        if ((-b - discriminant) / (2 * a) < 0) {
            return (-b + discriminant) / (2 * a);
        } else {
            return (-b - discriminant) / (2 * a);
        }
    }
}
