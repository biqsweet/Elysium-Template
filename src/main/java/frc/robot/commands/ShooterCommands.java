package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.RobotContainer.*;

public class ShooterCommands {
    public static Command receiveBall() {
        return INTAKE_ARM.setIntakeArmPosition(0).andThen(
                INTAKE.setIntakeVoltage(4).alongWith(
                        CONVEYOR.setConveyorVoltage(4)));
    }

    public static Command removeBall() {
        return INTAKE.setIntakeVoltage(-4).alongWith(
                CONVEYOR.setConveyorVoltage(-4)).andThen(
                        new WaitCommand(1.0)).andThen(
                                INTAKE.stop().alongWith(
                                        CONVEYOR.stop()));
    }

    public static Command aimAndShoot() {
        return ARM.setArmPosition(0.5).alongWith(
                TURRET.autoAimTurret().andThen(
                        FLYWHEEL.setFlywheelVoltage(-4))).andThen(
                                ARM.setArmPosition(0)).andThen(
                                        FLYWHEEL.stop().alongWith(
                                                ARM.stop()
                        )
                );
    }

    public static Command rotateTurret() {
        return TURRET.spinTurret();
    }

    public static Command stopAllSystems() {
        return ARM.stop().alongWith(
                TURRET.stop().alongWith(
                        INTAKE_ARM.stop().alongWith(
                                INTAKE.stop().alongWith(
                                        FLYWHEEL.stop().alongWith(
                                                CONVEYOR.stop())))));
    }
}
