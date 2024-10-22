package frc.robot.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.encoder.Encoder;
import frc.lib.generic.hardware.encoder.EncoderFactory;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.generic.hardware.motor.MotorFactory;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.generic.simulation.SimulationProperties;

public class TurretConstants extends GenericSubsystem {
    protected static final Motor TURRET_MOTOR = MotorFactory.createTalonFX("TURRET_MOTOR", 106);
    protected static final Encoder TURRET_ENCODER = EncoderFactory.createCanCoder("TURRET_MOTOR", 107);
    protected static final Motor SECOND_TURRET_MOTOR = MotorFactory.createTalonFX("SECOND_TURRET_MOTOR", 108);

    static {
        configureTurretMotor();
        setSimulatedEncoderSources();
    }

    private static void setSimulatedEncoderSources() {
        TURRET_ENCODER.setSimulatedEncoderPositionSource(TURRET_MOTOR::getSystemPosition);
        TURRET_ENCODER.setSimulatedEncoderVelocitySource(TURRET_MOTOR::getSystemVelocity);
    }

    private static void configureTurretMotor() {
        MotorConfiguration turretMotorConfiguration = new MotorConfiguration();

        SECOND_TURRET_MOTOR.setFollowerOf("TURRET_MOTOR", 107);

        turretMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;
        turretMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.SIMPLE_MOTOR, DCMotor.getFalcon500(2), 150, 0.02);

        TURRET_MOTOR.configure(turretMotorConfiguration);
    }
}
