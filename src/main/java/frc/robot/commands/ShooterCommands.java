package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.RobotContainer.CONVEYOR;
import static frc.robot.RobotContainer.INTAKE;

public class ShooterCommands {
    public static Command receiveBall(){
        return INTAKE.setIntakeVelocity()
                .alongWith()
                .alongWith();
    }
    public static Command moveBallToShooter(){
        return CONVEYOR.setConveyorVelocity()
                .alongWith();
    }
    public static Command Shoot(){
        return null;
    }

}
