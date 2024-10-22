package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.encoder.Encoder;
import frc.lib.generic.hardware.encoder.EncoderFactory;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.generic.hardware.motor.MotorFactory;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.generic.simulation.SimulationProperties;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.MAX;

public class IntakeArmConstants {
    protected static final Motor INTAKE_ARM_MOTOR = MotorFactory.createSpark("ARM_MOTOR", 104, MAX);
    protected static final Encoder INTAKE_ARM_ENCODER = EncoderFactory.createCanCoder("ARM_ENCODER", 105);

    private static final Rotation2d MAXIMUM_ANGLE = Rotation2d.fromDegrees(90);
    private static final Rotation2d MINIMUM_ANGLE = Rotation2d.fromDegrees(25);


    static {
        configureArmMotor();
        setSimulatedEncoderSources();
    }

    private static void setSimulatedEncoderSources() {
        INTAKE_ARM_ENCODER.setSimulatedEncoderPositionSource(INTAKE_ARM_MOTOR::getSystemPosition);
        INTAKE_ARM_ENCODER.setSimulatedEncoderVelocitySource(INTAKE_ARM_MOTOR::getSystemVelocity);
    }

    private static void configureArmMotor() {
        MotorConfiguration armMotorConfiguration = new MotorConfiguration();

        armMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;

        armMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.ARM, DCMotor.getFalcon500(1), 50, 0.5, 0.2, MINIMUM_ANGLE, MAXIMUM_ANGLE, true);

        INTAKE_ARM_MOTOR.configure(armMotorConfiguration);
    }
}
