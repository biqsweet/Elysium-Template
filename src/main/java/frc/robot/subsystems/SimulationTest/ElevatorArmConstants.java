package frc.robot.subsystems.SimulationTest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.encoder.Encoder;
import frc.lib.generic.hardware.encoder.EncoderFactory;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.simulation.mechanisms.ElevatorArmMechanism2d;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.MAX;

public class ElevatorArmConstants {
    protected static final Motor ARM_ELEVATOR_MOTOR = MotorFactory.createSpark("ARM_ELEVATOR_MOTOR", 500, MAX);
    protected static final Encoder ARM_ELEVATOR_ENCODER = EncoderFactory.createCanCoder("ARM_ELEVATOR_ENCODER", 501);

    protected static final Motor ELEVATOR_ARM_MOTOR = MotorFactory.createSpark("ELEVATOR_ARM_MOTOR", 506, MAX);
    protected static final Encoder ELEVATOR_ENCODER = EncoderFactory.createCanCoder("ELEVATOR_ARM_ENCODER", 507);

    protected static final ElevatorArmMechanism2d elevatorArmMechanism = new ElevatorArmMechanism2d("ElevatorArmMechanism2d", 4);

    protected static final Rotation2d
            MINIMUM_ROTATION = Rotation2d.fromDegrees(0),
            MAXIMUM_ROTATION = Rotation2d.fromDegrees(360);

    static {
        configureElevatorArmMotor();
        configureArmElevatorMotor();
    }

    private static void configureArmElevatorMotor() {
        final MotorConfiguration armElevatorMotorConfiguration = new MotorConfiguration();
        armElevatorMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;

        ARM_ELEVATOR_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        ARM_ELEVATOR_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        ARM_ELEVATOR_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        ARM_ELEVATOR_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);

        armElevatorMotorConfiguration.simulationSlot = new MotorProperties.Slot(100, 0, 0, 0, 0, 0);
        armElevatorMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.ARM, DCMotor.getFalcon500(1), 150, 0.5, 0.2, MINIMUM_ROTATION, MAXIMUM_ROTATION, true);

        ARM_ELEVATOR_MOTOR.configure(armElevatorMotorConfiguration);
    }

    private static void configureElevatorArmMotor() {
        final MotorConfiguration ElevatorArmMotorConfiguration = new MotorConfiguration();
        ElevatorArmMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;

        ELEVATOR_ARM_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        ELEVATOR_ARM_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        ELEVATOR_ARM_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        ELEVATOR_ARM_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);

        ElevatorArmMotorConfiguration.simulationSlot = new MotorProperties.Slot(100, 0, 0, 0, 0, 0);
        ElevatorArmMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.ELEVATOR, DCMotor.getFalcon500(1), 150, 0.25, 0.2, -100, 100, true);

        ELEVATOR_ARM_MOTOR.configure(ElevatorArmMotorConfiguration);
    }
}
