package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.RobotContainer.*;

public class ShooterCommands {
    public static Command receiveBall() {
        return INTAKE_ARM.engageIntakeArm().andThen(
                INTAKE.setIntakeVoltage(false).alongWith(
                        CONVEYOR.setConveyorVoltage(false).alongWith(
                                new WaitCommand(1.0).andThen(
                                        INTAKE_ARM.disengageIntakeArm().alongWith(
                                                INTAKE.stop().alongWith(
                                                        CONVEYOR.stop()
                                                )
                                        )
                                )
                        )
                )
        );
    }

    public static Command removeBall() {
        return INTAKE.setIntakeVoltage(true).alongWith(
                CONVEYOR.setConveyorVoltage(true).alongWith(
                        new WaitCommand(1.0).andThen(
                                INTAKE.stop().alongWith(
                                        CONVEYOR.stop()
                                )
                        )
                )
        );
    }

    public static Command aimAndShoot() {
        return ARM.setArmPosition45().alongWith(
                TURRET.autoAimTurret().andThen(
                        FLYWHEEL.setFlywheelVelocity().alongWith(
                                new WaitCommand(1.0).andThen(
                                        FLYWHEEL.stop().alongWith(
                                                TURRET.stop().alongWith(
                                                        ARM.stop()
                                                )
                                        )
                                )
                        )
                )
        );
    }

    public static Command armPositionTo10() {
        return ARM.setArmPosition10().andThen(
                ARM.stop());
    }
}
