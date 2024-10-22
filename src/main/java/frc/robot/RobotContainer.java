// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.util.Controller;
import frc.robot.commands.ShooterCommands;
import frc.robot.poseestimation.poseestimator.PoseEstimator;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeArm;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.verticalconveyor.VerticalConveyor;

import java.util.function.DoubleSupplier;

import static frc.lib.util.Controller.Axis.*;
import static frc.robot.poseestimation.poseestimator.PoseEstimatorConstants.FRONT_CAMERA;

public class RobotContainer {
    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator(FRONT_CAMERA);
    public static final Swerve SWERVE = new Swerve();
    public static final Intake INTAKE = new Intake();
    public static final IntakeArm INTAKE_ARM = new IntakeArm();
    public static final Arm ARM = new Arm();
    public static final Turret TURRET = new Turret();
    public static final Flywheel FLYWHEEL = new Flywheel();
    public static final VerticalConveyor CONVEYOR = new VerticalConveyor();

    public static final BuiltInAccelerometer ROBORIO_ACCELEROMETER = new BuiltInAccelerometer();

    private final Controller driveController = new Controller(0);
    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);

        driveController.getButton(Controller.Inputs.LEFT_BUMPER).whileTrue(ShooterCommands.receiveBall());
        driveController.getButton(Controller.Inputs.B).whileTrue(ShooterCommands.removeBall());
        driveController.getButton(Controller.Inputs.RIGHT_BUMPER).whileTrue(ShooterCommands.moveBallToShooter());
        // I don't know how to aim
        //pose estimator?

        DoubleSupplier translationSupplier = () -> MathUtil.applyDeadband(-driveController.getRawAxis(LEFT_Y), 0.05);
        DoubleSupplier strafeSupplier =() -> MathUtil.applyDeadband(-driveController.getRawAxis(LEFT_X), 0.05);
        DoubleSupplier turningSupplier =() -> MathUtil.applyDeadband(-driveController.getRawAxis(RIGHT_X), 0.05);

        SWERVE.setDefaultCommand(
                SWERVE.driveOpenLoop(
                        translationSupplier,
                        strafeSupplier,

                        turningSupplier,
                        () -> driveController.getStick(Controller.Stick.RIGHT_STICK).getAsBoolean()
                ));

        driveController.getButton(Controller.Inputs.START).whileTrue(SWERVE.resetGyro());
        driveController.getButton(Controller.Inputs.BACK).whileTrue(SWERVE.lockSwerve());
    }

    private void setupCharacterization(GenericSubsystem subsystem) {
        driveController.getButton(Controller.Inputs.A).whileTrue(subsystem.getSysIdQuastatic(SysIdRoutine.Direction.kForward));
        driveController.getButton(Controller.Inputs.B).whileTrue(subsystem.getSysIdQuastatic(SysIdRoutine.Direction.kReverse));
        driveController.getButton(Controller.Inputs.Y).whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kForward));
        driveController.getButton(Controller.Inputs.X).whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
