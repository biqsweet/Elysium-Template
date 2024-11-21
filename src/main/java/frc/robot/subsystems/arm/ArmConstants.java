package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.encoder.Encoder;
import frc.lib.generic.hardware.encoder.EncoderFactory;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.simulation.mechanisms.Component3d;
import frc.lib.generic.simulation.mechanisms.Object3d;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.MAX;

public class ArmConstants {
    protected static final Motor ARM_MOTOR = MotorFactory.createSpark("ARM_MOTOR", 102, MAX);
    protected static final Encoder ARM_ENCODER = EncoderFactory.createCanCoder("ARM_ENCODER", 103);

    protected static final Translation2d HUB_POSITION = new Translation2d(8, 4);

    protected static final Component3d ARM_POSE_3D = new Component3d(
            "ARM_MECHANISM",
            0.0054,
            0,
            0.5944
    );
    protected static final Object3d BALL = new Object3d(
            "BALL",
            0.0054,
            0,
            0.5944
    );

    private static final Rotation2d
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(90),
            MINIMUM_ANGLE = Rotation2d.fromDegrees(25);  //todo: consider splatting these together V

    static {
        configureArmMotor();
        setSimulatedEncoderSources();
    }

    private static void setSimulatedEncoderSources() {
        ARM_ENCODER.setSimulatedEncoderPositionSource(ARM_MOTOR::getSystemPosition);
        ARM_ENCODER.setSimulatedEncoderVelocitySource(ARM_MOTOR::getSystemVelocity);
    }

    private static void configureArmMotor() {
        final MotorConfiguration armMotorConfiguration = new MotorConfiguration();  //todo: FINAL!L! V

        armMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;

        ARM_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        ARM_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        ARM_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);

        armMotorConfiguration.simulationSlot = new MotorProperties.Slot(100, 0, 0, 0, 0, 0);
        armMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.ARM, DCMotor.getFalcon500(1), 150, 0.5, 0.2, MINIMUM_ANGLE, MAXIMUM_ANGLE, true);

        ARM_MOTOR.configure(armMotorConfiguration);
    }
}

