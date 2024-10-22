package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.RobotContainer.*;

public class ShooterCommands {
    public static Command receiveBall() {
        return INTAKE_ARM.engageIntakeArm().andThen(
                INTAKE.setIntakeVoltage(false).andThen(
                        new WaitCommand(1.0).andThen(
                                INTAKE_ARM.disengageIntakeArm()
                        )
                )
        );
    }

    public static Command removeBall() {
        return INTAKE.setIntakeVoltage(true);
    }

    public static Command moveBallToShooter() {
        return CONVEYOR.setConveyorVoltage();
    }

    public static Command aimAndShoot(Rotation2d armTargetPosition, Rotation2d turretTargetPosition) {
        return ARM.setArmPosition(armTargetPosition).alongWith(
                TURRET.setTurretPosition(turretTargetPosition)
        );
    }
}
