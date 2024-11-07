package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.encoder.Encoder;
import frc.lib.generic.hardware.encoder.EncoderFactory;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;

public class TurretConstants extends GenericSubsystem {
    protected static final Motor TURRET_MOTOR = MotorFactory.createTalonFX("TURRET_MOTOR", 106);
    protected static final Encoder TURRET_ENCODER = EncoderFactory.createCanCoder("TURRET_MOTOR", 107);
    protected static final Motor SECOND_TURRET_MOTOR = MotorFactory.createTalonFX("SECOND_TURRET_MOTOR", 108);

    protected static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(720);
    protected static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-720);
    protected static final double K_P = 40;

    protected static final double HUB_X = 8;
    protected static final double HUB_Y = 4;

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

        TURRET_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        TURRET_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);

        turretMotorConfiguration.simulationSlot = new MotorProperties.Slot(1, 0, 0, 0, 0, 0);
        turretMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;
        turretMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.SIMPLE_MOTOR, DCMotor.getFalcon500(2), 150, 0.02);
        turretMotorConfiguration.closedLoopContinuousWrap = true;

        TURRET_MOTOR.configure(turretMotorConfiguration);
    }
}
