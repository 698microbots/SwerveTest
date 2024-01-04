// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS; //used online installation, had to check for updates

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private final SwerveModuleBase frontLeft = new SwerveModuleBase(
    Constants.frontLeftTurn, 
    Constants.frontLeftDrive,
    false,
    false,
    Constants.frontLeftEncoder,
    0.0,
    false
  );
  
  private final SwerveModuleBase frontRight = new SwerveModuleBase(
    Constants.frontRightTurn, Constants.frontRightDrive,
    false,
    false,
    Constants.frontRightEncoder,
    0.0,
    false
  );

  private final SwerveModuleBase backLeft = new SwerveModuleBase(
    Constants.backLeftTurn, Constants.backLeftDrive,
    false,
    false,
    Constants.backRightEncoder,
    0.0,
    false
  );

  private final SwerveModuleBase backRight = new SwerveModuleBase(
    Constants.backRightTurn, Constants.frontLeftDrive,
    false,
    false,
    Constants.backRightEncoder,
    0.0,
    false
  );

  private final AHRS gyro = new AHRS(SPI.Port.kMXP); //uses MXP port not SPI for diret connection to roborio

  public DriveTrain() {
    new Thread(() -> { // run separately so the rest of the code can run? 
      try {
            Thread.sleep(1000);
            zeroHeading();
          } catch (Exception e){
          }



    }).start();
    


    
  }

  public void zeroHeading(){
    gyro.reset();
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getAngle(), 360);// remainder of gyro.getAngle/360
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }


  public void stopModules(){
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }


  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kPhysiycalMaxSpeedMetersPerSecond); //proportionally decreases wheel speeds so they can all be achieved
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
    
  }

  public void setModuleAngleToZero() { // sets and angle to 0
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Robot Heading", 0);
  }
}
