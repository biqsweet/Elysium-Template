package frc.robot.subsystems.SimulationTest;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.encoder.Encoder;
import frc.lib.generic.hardware.encoder.EncoderFactory;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.simulation.mechanisms.ElevatorMechanism2d;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.MAX;
import static frc.lib.generic.simulation.mechanisms.MechanismUtilities.createElevatorMechanism;

public class ElevatorConstants {
    protected static final Motor ELEVATOR_MOTOR = MotorFactory.createSpark("ELEVATOR_MOTOR", 506, MAX);
    protected static final Encoder ELEVATOR_ENCODER = EncoderFactory.createCanCoder("ELEVATOR_ENCODER", 507);

    protected static final ElevatorMechanism2d elevatorMechanism = createElevatorMechanism("ElevatorMechanism2d", 4);

    static {
        configureElevatorMotor();
    }

    private static void configureElevatorMotor() {
        final MotorConfiguration ElevatorMotorConfiguration = new MotorConfiguration();
        ElevatorMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;

        ELEVATOR_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        ELEVATOR_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        ELEVATOR_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        ELEVATOR_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);

        ElevatorMotorConfiguration.simulationSlot = new MotorProperties.Slot(100, 0, 0, 0, 0, 0);
        ElevatorMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.ELEVATOR, DCMotor.getFalcon500(1), 150, 0.25, 0.2, -100, 100, true);

        ELEVATOR_MOTOR.configure(ElevatorMotorConfiguration);
    }
}
