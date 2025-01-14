package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.visualization.mechanisms.SpeedMechanism2d;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.MAX;
import static frc.lib.generic.visualization.mechanisms.MechanismFactory.createSpeedMechanism;

public class FlywheelConstants {
    protected static final Motor FLYWHEEL_MOTOR = MotorFactory.createSpark("TEST_FLYWHEEL_MOTOR", 508, MAX);
    protected static final SpeedMechanism2d flywheelMechanism = createSpeedMechanism("SpeedMechanism2d");

    static {
        configureFlywheelMotor();
    }

    private static void configureFlywheelMotor() {
        final MotorConfiguration flywheelMotorConfiguration = new MotorConfiguration();
        flywheelMotorConfiguration.idleMode = MotorProperties.IdleMode.COAST;

        FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);

        flywheelMotorConfiguration.simulationSlot = new MotorProperties.Slot(10, 0, 0, 0, 0, 0);
        flywheelMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.SIMPLE_MOTOR, DCMotor.getFalcon500(1), 150, 0.2);

        FLYWHEEL_MOTOR.configure(flywheelMotorConfiguration);
    }
}
