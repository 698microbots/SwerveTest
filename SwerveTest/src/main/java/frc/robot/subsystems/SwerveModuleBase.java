// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


public class SwerveModuleBase extends SubsystemBase {
  
  private final TalonFX motorTurn;
  private final TalonFX motorDrive;

  private final double motorTurnEncoder;
  private final double motorDriveEncoder;
  
  private final AnalogInput absoluteEncoder; //in order to monitor encoder positions after turning off
  private final boolean absoluteEncoderReversed;
  private double absoluteEncoderOffsetRad;
  
  private final PIDController turningPidController;
  
  /** Creates a new SwerveDrivetrain. */
  public SwerveModuleBase(int motorTurnID, int motorDriveID, boolean driveMotorReversed, boolean turningMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    
    absoluteEncoder = new AnalogInput(absoluteEncoderId);

    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;

    motorTurn = new TalonFX(motorTurnID);
    motorDrive = new TalonFX(motorDriveID);

    motorTurn.setInverted(turningMotorReversed);
    motorDrive.setInverted(driveMotorReversed);

    motorTurnEncoder = motorTurn.getSelectedSensorPosition();
    motorDriveEncoder = motorDrive.getSelectedSensorPosition(); // need to set some conversions later

    turningPidController = new PIDController(Constants.kPTurning, 0, 0);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public double getDrivePosition(){
    return motorDrive.getSelectedSensorPosition();
  }

  public double getTurningPosition(){
    return motorTurn.getSelectedSensorPosition();
  }

  public double getDriveVelocity(){
    return motorDrive.getSelectedSensorVelocity();
  }

  public double getTurnVelocity(){
    return motorTurn.getSelectedSensorVelocity();
  }

  public double getAbsoluteEncoderRad(){
    double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    angle = angle * 2.0 * Math.PI;
    if (absoluteEncoderReversed){
      return angle * -1;
    } else {
      return angle;
    }
  }

  public void resetEncoders(){
    motorDrive.setSelectedSensorPosition(0);
    motorTurn.setSelectedSensorPosition(getAbsoluteEncoderRad()); // turning encoders will be aligned with the angle
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state){
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle); // optimize the turning so that for example instead of turning 270deg turns -90deg
    motorDrive.set(TalonFXControlMode.Velocity, state.speedMetersPerSecond / Constants.kPhysiycalMaxSpeedMetersPerSecond);
    motorTurn.set(TalonFXControlMode.Velocity, turningPidController.calculate(getTurningPosition(),state.angle.getRadians()));
    SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
  }

  public void stop() {
    motorDrive.set(TalonFXControlMode.Velocity,0);
    motorTurn.set(TalonFXControlMode.Velocity,0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
