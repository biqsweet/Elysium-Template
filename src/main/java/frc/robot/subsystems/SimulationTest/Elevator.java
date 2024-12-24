package frc.robot.subsystems.SimulationTest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.SimulationTest.ElevatorConstants.ELEVATOR_MOTOR;
import static frc.robot.subsystems.SimulationTest.ElevatorConstants.elevatorMechanism;

public class Elevator extends GenericSubsystem {
    /**
     * @param position in rotations
     */
    public Command setElevatorPosition(double position) {
        return Commands.run(() -> ELEVATOR_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, position), this);
    }

    public Command stopElevator() {
        return Commands.runOnce(ELEVATOR_MOTOR::stopMotor, this);
    }

    public double getCurrentPosition() {
        return ELEVATOR_MOTOR.getSystemPosition();
    }

    public double getTargetPosition() {
        return ELEVATOR_MOTOR.getClosedLoopTarget();
    }

    @Override
    public void periodic() {
        elevatorMechanism.updateCurrentPosition(getTargetPosition() * 10);
        elevatorMechanism.updateTargetPosition(getCurrentPosition() * 10);
    }
}
