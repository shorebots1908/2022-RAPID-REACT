// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* TODOs
!TURN FOR TARGET ACQUISITION
!TOGGLE DIRECTION OF TURN FOR TARGETING
*SPIN UP AND SHOOT WHEN IN RANGE
*USE ENCODERS TO DETERMINE WHEN TO FEED?
*PUT CONFIGURATION FOR LIMELIGHT SEEKING ON SMART DASHBOARD
*/

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

// import javax.management.relation.Relation;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.RelativeEncoder;
//import com.revrobotics.spark.config.SparkBaseConfig;
//import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class Robot extends TimedRobot {

  private XboxController xBox = new XboxController(0);
  //private final SendableChooser<String> reelChooser = new SendableChooser<>();
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private WPI_TalonSRX motorL1 = new WPI_TalonSRX(1);
  private WPI_TalonSRX motorR1 = new WPI_TalonSRX(2);
  private SparkMax outMotor = new SparkMax(9, MotorType.kBrushless);
  private SparkMax outMotor2 = new SparkMax(10, MotorType.kBrushless);
  private DifferentialDrive driveRobot;
  private SlewRateLimiter driveAccLimiter = new SlewRateLimiter(3);
  private Spark trigger = new Spark(1);
  NetworkTable FMS = NetworkTableInstance.getDefault().getTable("FMSInfo");
 
  //auto variable programs
  //control variables
  private double inputScaling = 0.6;
  private int povState = -1;
  private double oldSlew = 3.0;


  //configuration variables
  private double feedSpeed = -1;
  private double highSpeed = -0.75;

  @Override
  public void robotInit() 
  {
    SmartDashboard.putNumber("Slew Rate", 3.0);
    SmartDashboard.putData("Auto choices", m_chooser);
    driveRobot = new DifferentialDrive(motorL1::set,motorR1::set);
    SparkBaseConfig outMotor2Config = new SparkMaxConfig().follow(outMotor, /*invert*/ false);
    outMotor2.configure(outMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void robotPeriodic() 
  {
    SmartDashboard.updateValues();
    if(SmartDashboard.getNumber("Slew Rate", 3) != oldSlew)
    {
      driveAccLimiter = new SlewRateLimiter(SmartDashboard.getNumber("Slew Rate", 3));
      oldSlew = SmartDashboard.getNumber("Slew Rate", 3);
    }

  }

  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }
    

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }
//#endregion
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
      driveRobot.arcadeDrive((-xBox.getLeftX() * inputScaling),driveAccLimiter.calculate(xBox.getLeftY() * inputScaling));

    // use the DPAD to determine robot speeds
    if (povState != xBox.getPOV()) {
      povState = xBox.getPOV();
      switch (povState) {
          case 180 -> inputScaling = 0.4;
          case 0   -> inputScaling = 0.7;
          case 270 -> inputScaling = 0.55;
          case 90  -> inputScaling = 1.0;
      }
  }
        
    driveRobot.setMaxOutput(1.0 - xBox.getLeftTriggerAxis());

    if(xBox.getRightBumperButton())
    {
      outMotor.set(highSpeed);
    }
    else
    {
      outMotor.stopMotor();
    }
    
    if(xBox.getAButton())
    {
      trigger.set(feedSpeed);
    }
    else
    {
      trigger.stopMotor();
    }

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit()
  {

  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() 
  {

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic()
  {

  }
}
