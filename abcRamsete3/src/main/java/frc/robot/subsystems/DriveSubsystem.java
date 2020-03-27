/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.Constants.DriveConstants;



public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.


  WPI_TalonSRX m_leftmaster = new WPI_TalonSRX(DriveConstants.kLeftMotor1Port);
  WPI_TalonSRX m_rightmaster = new WPI_TalonSRX(DriveConstants.kRightMotor1Port);

  WPI_TalonSRX m_leftslave0 = new WPI_TalonSRX(DriveConstants.kLeftMotor2Port);
  WPI_TalonSRX m_rightslave0 = new WPI_TalonSRX(DriveConstants.kRightMotor2Port);
  
  double encoderConstant = ((DriveConstants.kWheelDiameterMeters * Math.PI) / DriveConstants.kEncoderDistancePerPulse);

  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(m_leftmaster,m_leftslave0);

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(m_rightmaster,m_rightslave0);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);


  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
 
    resetEncoders();
    drivetrainInit();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), 
                      getLeftMasterPosition(),
                      getRightMasterRate());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftMasterRate(), getRightMasterRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_leftmaster.setSelectedSensorPosition(0);
    m_rightmaster.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (getLeftMasterPosition()+ getRightMasterPosition()) / 2.0;
  }


public double getLeftMasterPosition ()   {
  return (m_leftmaster.getSelectedSensorPosition(0)/DriveConstants.kEncoderDistancePerPulse);
}

public double getRightMasterPosition ()   {
  return (m_rightmaster.getSelectedSensorPosition(0)/DriveConstants.kEncoderDistancePerPulse);
}

public double getLeftMasterRate ()   {
  return (m_leftmaster.getSelectedSensorVelocity(0) /DriveConstants.kEncoderDistancePerPulse * 10);
}


public double getRightMasterRate ()   {
  return (m_rightmaster.getSelectedSensorVelocity(0)/DriveConstants.kEncoderDistancePerPulse * 10);
}


  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void drivetrainInit () {
    m_leftmaster.setInverted(DriveConstants.kLeftMotorInverted);
    m_leftmaster.setSensorPhase(DriveConstants.kLeftEncoderReversed);
    m_leftmaster.setNeutralMode(NeutralMode.Brake);

    m_leftslave0.setInverted(DriveConstants.kLeftMotorInverted);
    m_leftslave0.setSensorPhase(DriveConstants.kLeftEncoderReversed);
    m_leftslave0.setNeutralMode(NeutralMode.Brake);

    m_rightmaster.setInverted(DriveConstants.kRightMotorInverted);
    m_rightmaster.setSensorPhase(DriveConstants.kRightEncoderReversed);
    m_rightmaster.setNeutralMode(NeutralMode.Brake);

    m_rightslave0.setInverted(DriveConstants.kRightMotorInverted);
    m_rightslave0.setSensorPhase(DriveConstants.kRightEncoderReversed);
    m_rightslave0.setNeutralMode(NeutralMode.Brake);
  }
}
