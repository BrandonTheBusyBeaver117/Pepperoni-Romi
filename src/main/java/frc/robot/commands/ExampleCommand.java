// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ExampleCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RomiDrivetrain driveSubsystem;
  DoubleSupplier rightTriggerSpeedSupplier;
  DoubleSupplier leftTriggerSpeedSupplier;
  DoubleSupplier leftJoystickRotationSupplier;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand(RomiDrivetrain driveSubsystem, DoubleSupplier triggerSpeedSupplier, DoubleSupplier leftTriggerSpeedSupplier, DoubleSupplier leftJoystickRotationSupplier) {
    this.driveSubsystem = driveSubsystem;
    this.rightTriggerSpeedSupplier = triggerSpeedSupplier;
    this.leftTriggerSpeedSupplier = leftTriggerSpeedSupplier;
    this.leftJoystickRotationSupplier = leftJoystickRotationSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = rightTriggerSpeedSupplier.getAsDouble() - leftTriggerSpeedSupplier.getAsDouble();
    double rotation = leftJoystickRotationSupplier.getAsDouble();

    //driveSubsystem.arcadeDrive(speed, rotation);
    driveSubsystem.tankDrive(speed, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
