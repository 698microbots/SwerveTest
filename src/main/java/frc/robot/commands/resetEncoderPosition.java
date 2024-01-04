// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.SwerveModuleBase;

public class resetEncoderPosition extends CommandBase {

  private DriveTrain swerveModuleBase;
  /** Creates a new resetEncoderPosition. */
  public resetEncoderPosition(DriveTrain swerveModuleBase) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveModuleBase = swerveModuleBase;
    addRequirements(swerveModuleBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { // set the speed and angle for all to 0
    swerveModuleBase.stopModules();
    swerveModuleBase.setModuleAngleToZero();
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
