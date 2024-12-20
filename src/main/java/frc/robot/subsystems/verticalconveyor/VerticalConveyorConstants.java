package frc.robot.subsystems.verticalconveyor;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.MAX;

public class VerticalConveyorConstants {
    protected static final Motor CONVEYOR_MOTOR = MotorFactory.createSpark("CONVEYOR_MOTOR", 101, MAX);

    static {
        configureConveyor();
    }

    private static void configureConveyor() {
        final MotorConfiguration conveyorMotorConfiguration = new MotorConfiguration();

        CONVEYOR_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);

        conveyorMotorConfiguration.idleMode = MotorProperties.IdleMode.COAST;
        conveyorMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.SIMPLE_MOTOR, DCMotor.getFalcon500(1), 150, 0.02);
        conveyorMotorConfiguration.simulationSlot = new MotorProperties.Slot(1, 0, 0, 0, 0, 0);

        CONVEYOR_MOTOR.configure(conveyorMotorConfiguration);
    }
}