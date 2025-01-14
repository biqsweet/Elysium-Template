package frc.robot.subsystems.doubleJointedArm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.encoder.Encoder;
import frc.lib.generic.hardware.encoder.EncoderFactory;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.visualization.mechanisms.DoubleJointedArmMechanism2d;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.MAX;
import static frc.lib.generic.visualization.mechanisms.MechanismFactory.createDoubleJointedArmMechanism;

public class DoubleJointedArmConstants {
    protected static final Motor
            SHOULDER_MOTOR = MotorFactory.createSpark("SHOULDER_MOTOR", 502, MAX),
            ELBOW_MOTOR = MotorFactory.createSpark("ELBOW_MOTOR", 503, MAX);

    protected static final Encoder
            SHOULDER_ENCODER = EncoderFactory.createCanCoder("SHOULDER_ENCODER", 504),
            ELBOW_ENCODER = EncoderFactory.createCanCoder("ELBOW_ENCODER", 505);

    protected static final DoubleJointedArmMechanism2d doubleJointedArmMechanism2d = createDoubleJointedArmMechanism("DoubleJointedArmMechanism2d", 3, 2);

    static {
        configureShoulderMotor();
        configureElbowMotor();
    }

    private static void configureShoulderMotor() {
        final MotorConfiguration shoulderMotorConfiguration = new MotorConfiguration();
        shoulderMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;

        SHOULDER_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        SHOULDER_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        SHOULDER_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        SHOULDER_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);

        shoulderMotorConfiguration.simulationSlot = new MotorProperties.Slot(100, 0, 0, 0, 0, 0);
        shoulderMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.ARM, DCMotor.getFalcon500(1), 150, 0.5, 0.2, Rotation2d.fromDegrees(-360), Rotation2d.fromDegrees(360), true);

        SHOULDER_MOTOR.configure(shoulderMotorConfiguration);
    }

    private static void configureElbowMotor() {
        final MotorConfiguration elbowMotorConfiguration = new MotorConfiguration();
        elbowMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;

        ELBOW_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        ELBOW_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        ELBOW_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        ELBOW_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);

        elbowMotorConfiguration.simulationSlot = new MotorProperties.Slot(100, 0, 0, 0, 0, 0);
        elbowMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.ARM, DCMotor.getFalcon500(1), 150, 0.5, 0.2, Rotation2d.fromDegrees(-360), Rotation2d.fromDegrees(360), true);

        ELBOW_MOTOR.configure(elbowMotorConfiguration);
    }
}
