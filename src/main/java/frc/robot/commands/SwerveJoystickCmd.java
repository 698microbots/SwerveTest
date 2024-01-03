// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SOURCE : https://www.youtube.com/watch?v=0Xi9yb1IMyA
 // left off at 13:04 and idnt make the deadband
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.SwerveModuleBase;

public class SwerveJoystickCmd extends CommandBase {
  /** Creates a new SwerveJoystickCmd. */
  private final DriveTrain swerve;
  private final Supplier<Double> x, y, turning;
  private final Supplier<Boolean> fieldOrientedFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  public SwerveJoystickCmd(DriveTrain swerve, Supplier<Double> x, 
  Supplier<Double> y, Supplier<Double> turning, Supplier<Boolean> fieldOrientedFunction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.x = x;
    this.y = y;
    this.turning = turning;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.xLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get realtime joystick inputs
    double xSpeed = x.get();
    double ySpeed = y.get();
    double turningSpeed = turning.get();
    //DID NOT ADD A DEADBAND (IGNORES SMALL JOYSTICK VALUES WHEN TRYING TO CENTER BACK TO 0)
    //Rate limiter to limit acceleration, make robot drive smoother.
    
    xSpeed = xLimiter.calculate(xSpeed) * Constants.kTeleDriveMaxSpeedMetersPerSecond;
    xSpeed = yLimiter.calculate(xSpeed) * Constants.kTeleDriveMaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed) * Constants.kTeleDriveMaxAngularSpeedUnitsPerSecond;    

    ChassisSpeeds chassisSpeeds;
    if (fieldOrientedFunction.get()){
      //relative to field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerve.getRotation2d());
    } else {
      //relative to forward direction
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    //convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerve.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
