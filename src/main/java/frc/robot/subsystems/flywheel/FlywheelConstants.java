package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.MAX;

public class FlywheelConstants {
    protected static final Motor FLYWHEEL_MOTOR = MotorFactory.createSpark("FLYWHEEL_MOTOR", 109, MAX);

    static {
        configureFlywheelMotor();
    }

    private static void configureFlywheelMotor() {
        MotorConfiguration flywheelMotorConfiguration = new MotorConfiguration();

        FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);

        flywheelMotorConfiguration.simulationSlot = new MotorProperties.Slot(1, 0, 0, 0, 0, 0);
        flywheelMotorConfiguration.idleMode = MotorProperties.IdleMode.COAST;
        flywheelMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.FLYWHEEL, DCMotor.getFalcon500(1), 150, 0.02);

        FLYWHEEL_MOTOR.configure(flywheelMotorConfiguration);
    }
}
