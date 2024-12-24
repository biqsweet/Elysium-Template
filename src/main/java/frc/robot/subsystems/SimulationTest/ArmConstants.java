package frc.robot.subsystems.SimulationTest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.encoder.Encoder;
import frc.lib.generic.hardware.encoder.EncoderFactory;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.simulation.mechanisms.SingleJointedArmMechanism2d;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.MAX;

public class ArmConstants {
    protected static final Motor ARM_MOTOR = MotorFactory.createSpark("ARM_MOTOR", 500, MAX);
    protected static final Encoder ARM_ENCODER = EncoderFactory.createCanCoder("ARM_ENCODER", 501);

    protected static final Rotation2d
            MINIMUM_ROTATION = Rotation2d.fromDegrees(0),
            MAXIMUM_ROTATION = Rotation2d.fromDegrees(360);

    protected static final SingleJointedArmMechanism2d armMechanism = new SingleJointedArmMechanism2d("SingleJointedArmMechanism2d", 10);

    static {
        configureArmMotor();
    }

    private static void configureArmMotor() {
        final MotorConfiguration armMotorConfiguration = new MotorConfiguration();
        armMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;

        ARM_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        ARM_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        ARM_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        ARM_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);

        armMotorConfiguration.simulationSlot = new MotorProperties.Slot(100, 0, 0, 0, 0, 0);
        armMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.ARM, DCMotor.getFalcon500(1), 150, 0.5, 0.2, MINIMUM_ROTATION, MAXIMUM_ROTATION, true);

        ARM_MOTOR.configure(armMotorConfiguration);
    }
}
