package frc.robot.subsystems.verticalconveyor;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.generic.hardware.motor.MotorFactory;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.generic.simulation.SimulationProperties;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.MAX;

public class VerticalConveyorConstants {
    protected static final Motor CONVEYOR_MOTOR = MotorFactory.createSpark("CONVEYOR_MOTOR", 101, MAX);

    static {
        configureConveyor();
    }

    private static void configureConveyor() {
        MotorConfiguration conveyorMotorConfiguration = new MotorConfiguration();

        conveyorMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;
        conveyorMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.SIMPLE_MOTOR, DCMotor.getFalcon500(1), 150, 0.02);


        CONVEYOR_MOTOR.configure(conveyorMotorConfiguration);
    }
}