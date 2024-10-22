package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.RobotContainer.*;

public class ShooterCommands {
    public static Command receiveBall(){
        return INTAKE.setIntakeVelocity();
    }

    public static Command removeBall() {
        return INTAKE.setIntakeVelocity(true);
    }

    public static Command moveBallToShooter() {
        return CONVEYOR.setConveyorVelocity();
    }

    public static Command AimAndShoot(Rotation2d armTargetPosition, Rotation2d turretTargetPosition) {
        return ARM.setArmPosition(armTargetPosition).alongWith(
                TURRET.setTurretPosition(turretTargetPosition)
        );
    }
}
