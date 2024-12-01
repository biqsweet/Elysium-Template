package frc.robot.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import static frc.lib.math.Conversions.rpsToMps;
import static frc.robot.GlobalConstants.GRAVITY_FORCE;
import static frc.robot.GlobalConstants.HUB_POSITION;
import static frc.robot.RobotContainer.*;

public class ShootBall extends Command {
    private double phi;
    private double theta;
    private Pose2d robotPosition;
    private Pose3d ballPosition;

    @Override
    public void initialize() {
        this.phi = ARM.getCurrentArmPosition().getDegrees();
        this.theta = TURRET.getCurrentTurretPosition().getDegrees();
        this.robotPosition = POSE_ESTIMATOR.getCurrentPose();
        this.ballPosition = new Pose3d(
                robotPosition.getX(), robotPosition.getY(), 1,
                new Rotation3d(0, 0, 0)
        );
    }

    @Override
    public void execute() {
        this.ballPosition = ballPosition.plus(new Transform3d(
                ballTrajectory().getX(),
                ballTrajectory().getY(),
                ballTrajectory().getZ(),
                new Rotation3d(0,0,0)));
        Logger.recordOutput("BALL", ballPosition);
    }

    @Override
    public boolean isFinished() {
        return ballPosition.getZ() <= 0;
    }

    public Translation3d ballTrajectory() {
        final double F = rpsToMps(FLYWHEEL.getCurrentVelocity().getRotations(), 3) / 100;

        final double z = F * Math.sin(phi);
        final double x = F * Math.cos(phi) * Math.cos(theta);
        final double y = F * Math.cos(phi) * Math.sin(theta);

        Translation2d distanceToHub = HUB_POSITION.toTranslation2d().minus(robotPosition.getTranslation());
        final double acceleration = GRAVITY_FORCE / 100;
        final double time = findTime(F, distanceToHub.getNorm());

        return new Translation3d(x, y, z - acceleration);
    }

    /**
     * @param startingV starting velocity
     * @param distance  deltaX
     */
    private double findTime(double startingV, double distance) {
        // the startV is a vector, the x speed is constant but the y speed is going down cuz of gravity
        return quadraticFormula(GRAVITY_FORCE / 100, startingV, distance);
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