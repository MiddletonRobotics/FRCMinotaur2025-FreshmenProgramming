// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  private WPI_TalonSRX leftFront;
  private WPI_TalonSRX rightFront;
  private WPI_TalonSRX leftRear;
  private WPI_TalonSRX rightRear;

  private DifferentialDrive drivetrain;

  private XboxController driverController;

  public Robot() {
    leftFront = new WPI_TalonSRX(1);
    rightFront = new WPI_TalonSRX(2);
    leftRear = new WPI_TalonSRX(3);
    rightRear = new WPI_TalonSRX(4);

    leftFront.setInverted(true);
    leftRear.setInverted(true);
    rightFront.setInverted(false);
    rightRear.setInverted(false);

    leftFront.setNeutralMode(NeutralMode.Brake);
    leftRear.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    rightRear.setNeutralMode(NeutralMode.Brake);

    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    drivetrain = new DifferentialDrive(leftFront, rightFront);
    drivetrain.setSafetyEnabled(true);

    driverController = new XboxController(0);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    drivetrain.arcadeDrive(driverController.getLeftY(), driverController.getLeftX());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
