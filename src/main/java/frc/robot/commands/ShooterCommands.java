package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.RobotContainer.*;

public class ShooterCommands {
    public static Command receiveBall() {
        return INTAKE_ARM.setIntakeArmPosition(0).andThen(
                INTAKE.setIntakeVoltage(4).alongWith( //todo: Seems like a lot of these and these could be completed in parallel
                        CONVEYOR.setConveyorVoltage(4)));
    }//todo: If you see staircases you're probably doing something wrong. Split these into multiple Command variables and call them instead of this mess

    public static Command removeBall() {
        return INTAKE.setIntakeVoltage(-4).alongWith(
                CONVEYOR.setConveyorVoltage(-4)).andThen(
                new WaitCommand(1.0)).andThen(
                INTAKE.stop().alongWith(CONVEYOR.stop()));
    }//todo: Again, no staircases. Also do it in parallel

    public static Command aimAndShoot() {
        return ARM.setArmPosition(0.5).andThen(
                FLYWHEEL.setFlywheelVoltage(-4)).andThen(
                ARM.setArmPosition(0)).andThen(
                FLYWHEEL.stop().alongWith(
                        ARM.stop()));
    }//todo: Parallel? no staircase

    public static Command rotateTurret() {//todo: Better name please
        return TURRET.spinTurret();
    }

    public static Command stopAllSystems() {
        return ARM.stop().alongWith(
                TURRET.stop()).alongWith(
                INTAKE_ARM.stop()).alongWith(
                INTAKE.stop()).alongWith(
                FLYWHEEL.stop()).alongWith(
                CONVEYOR.stop());
    }//todo: Staircase, parallel
}
