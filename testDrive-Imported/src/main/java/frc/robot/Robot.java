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
//import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
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
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class Robot extends TimedRobot {
  private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  private double startTime;
  private static final String kDefaultAuto = "2 ball high shooter #2 (middle)";
  private static final String kCustomAuto = "Low goal shooter";
  private static final String kCustomAuto2 = "High goal shooter";
  private static final String kCustomAuto3 = "2 ball high shooter #3 (wall)";
  private static final String kCustomAuto4 = "2 ball high shooter #1 (left)";
  private static final String kCustomAuto5 = "2 ball low shooter #1 (left)";
  private static final String kCustomAuto6 = "2 ball low shooter #2 (middle)";
  private String m_autoSelected;
  private XboxController xBox = new XboxController(0);
  private Joystick joystick = new Joystick(1);
  //private final SendableChooser<String> reelChooser = new SendableChooser<>();
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final SendableChooser<String> limelightchooser = new SendableChooser<>();
  private final String left = "left";
  private final String right = "right";
  //private Joystick stickL = new Joystick(0);
  //private Joystick stickR = new Joystick(1);
  private SparkMax motorL1 = new SparkMax(10, MotorType.kBrushless);
  private SparkMax motorL2 = new SparkMax(11, MotorType.kBrushless);
  private SparkMax motorR1 = new SparkMax(12, MotorType.kBrushless);
  private SparkMax motorR2 = new SparkMax(13, MotorType.kBrushless);
  private SparkMax inMotor = new SparkMax(7, MotorType.kBrushless);
  private SparkMax outMotor = new SparkMax(9, MotorType.kBrushless);
  private SparkMax feedMotor = new SparkMax(8, MotorType.kBrushless);
  private SparkMax reelMotor = new SparkMax(6, MotorType.kBrushless);
  private boolean reelCoast = false;
  private boolean startReleased = false;
  private DifferentialDrive driveRobot;
  //private MotorControllerGroup leftGroup;
  //private MotorControllerGroup rightGroup;
  //private final I2C.Port i2cPort = I2C.Port.kOnboard;
  //private ColorSensorV3 ColorSensor = new ColorSensorV3(i2cPort);
  private AnalogInput feederSensor = new AnalogInput(2);
  private AnalogInput preFeedSensor = new AnalogInput(3);
//   private AnalogInput distanceSensor = new AnalogInput(2);
  private SlewRateLimiter driveAccLimiter = new SlewRateLimiter(3);
  // private double heading;
  private Spark ledStrip = new Spark(0);
  private double green = 0.71;
  //private double teamColor;
  private boolean isRedAlliance = false;
  NetworkTable FMS = NetworkTableInstance.getDefault().getTable("FMSInfo");
  //auto variable programs
  //control variables
  private double inputScaling = 0.4;
  private int povState = -1;
  private double oldSlew = 3.0;
  //private int speedIndex = 0;
  private boolean aToggleState = false, bToggleState = false;
  private double timePassed;
  private double driveDuration, driveBackStart, shootTime;
  private double highFeedStart, lowFeedStart;
  private boolean feedFlag = false;
  private String mode;
  private double reverseDelay;
  private double autoFeedTimeStart;

  //configuration variables
  private double inSpeed = -0.7;
  private double feedSpeed = 0.6;
  private double highSpeed = 0.9;
  private double lowSpeed = 0.45; 

  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;
  private String ll_TurnDirection;

  //Custom Functions

  public void Update_Limelight_Tracking()
  {
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.045;                    // how hard to turn toward the target
        final double DRIVE_K = 0.06;                    // how hard to drive fwd toward the target
        //final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast
        final double MIN_DRIVE = 0.35;                  //minimum drive speed so things don't peter out or take too long.
        final double MIN_STEER = 0.3;
        final double MAX_STEER = 0.65;
        final double X_OFFSET_TARGET = 2;        //target target X deviation
        final double Y_OFFSET_TARGET = 2;         //target target Y deviation
        final double STEER_X_TARGET = X_OFFSET_TARGET * STEER_K;
        final double DRIVE_Y_TARGET = Y_OFFSET_TARGET * DRIVE_K;


        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        if (tv < 1.0)
        {
          m_LimelightHasValidTarget = false;
          m_LimelightDriveCommand = 0.0;
          m_LimelightSteerCommand = 0.0;
          return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        if((steer_cmd < STEER_X_TARGET) && (steer_cmd > -STEER_X_TARGET))
        {
          steer_cmd = 0;
        }
        else if((steer_cmd < MIN_STEER) && (steer_cmd > -MIN_STEER))
        {
          //if steer_cmd is > 0, make it 0.3, otherwise make it -0.3
          steer_cmd = steer_cmd > 0 ? MIN_STEER : -MIN_STEER;
        }
        else if((steer_cmd > MAX_STEER) || (steer_cmd < -MAX_STEER))
        {
          steer_cmd = steer_cmd > 0 ? MAX_STEER : -MAX_STEER;
        }

        m_LimelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        //double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
        
        // try to drive forward until the target is high enough on the view
        double drive_cmd = -ty * DRIVE_K;
        if((drive_cmd < DRIVE_Y_TARGET) && (drive_cmd > -DRIVE_Y_TARGET))
        {
          drive_cmd = 0;
        }
        else if((drive_cmd < MIN_DRIVE) && (drive_cmd > -MIN_DRIVE))
        {
          //if drive_cmd is > 0, make it 0.3, otherwise make it -0.3
          drive_cmd = drive_cmd > 0 ? MIN_DRIVE : -MIN_DRIVE;
        }

        // don't let the robot drive too fast into the goal
        if ((drive_cmd > MAX_DRIVE) || drive_cmd < -MAX_DRIVE)
        {
          drive_cmd = drive_cmd > 0 ? MAX_DRIVE : -MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;

        SmartDashboard.putNumber("LimelightX", tx);
        SmartDashboard.putNumber("LimelightY", ty);
        SmartDashboard.putNumber("LimelightArea", ta);
        SmartDashboard.putNumber("LimelightV", tv);

  }

  public double getTeamColor(){    
    isRedAlliance = FMS.getEntry("IsRedAlliance").getBoolean(false);
    if(isRedAlliance)
    {
      return 0.61;
    }
    else
    {
      return 0.87;
    }    
  }

  private void autoFeedRoutine()
  {
    if(feederSensor.getValue()>= 300 || (preFeedSensor.getValue() < 800 && (Timer.getFPGATimestamp() - autoFeedTimeStart) >= 3))
    {
      feedFlag=false;
    }

    if(preFeedSensor.getValue() >= 800 && feederSensor.getValue() < 300 && (Timer.getFPGATimestamp() - reverseDelay > 2) && !xBox.getXButton())
    {
      autoFeedTimeStart = Timer.getFPGATimestamp();
      feedFlag = true;
    }

    if(feedFlag && !(xBox.getLeftBumperButton() || xBox.getRightBumperButton() || xBox.getXButton() || xBox.getYButton()))
    {
      feedMotor.set(0.3);
    } 
    else if(!feedFlag && !(xBox.getLeftBumperButton() || xBox.getRightBumperButton() || xBox.getXButton() || xBox.getYButton()))
    {
      feedMotor.stopMotor();
    }
  }

  private void intakeReel()
  {
    double reelRevs = SmartDashboard.getNumber("Reel Revolutions", 9);
    double reelPosition = reelMotor.getEncoder().getPosition();
    if(reelCoast)
    {
     reelMotor.configure(
        new SparkMaxConfig().idleMode(IdleMode.kCoast),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    }
    else if(startReleased)
    {
    //reelMotor.setIdleMode(IdleMode.kBrake);
    reelMotor.configure(
        new SparkMaxConfig().idleMode(IdleMode.kBrake),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters); 
    reelMotor.getEncoder().setPosition(0);
    }
    else if(bToggleState && reelPosition < 0.75 * reelRevs)
    {
      reelMotor.set(0.3);
    }
    else if(bToggleState && reelPosition < reelRevs)
    {
      reelMotor.set(0.2);
    }
    else if(bToggleState)
    {
      reelMotor.stopMotor();
    }
    else if(!bToggleState && reelPosition > 0.25 * reelRevs)
    {
      reelMotor.set(-0.3);
    }
    else if(!bToggleState && reelPosition > 0)
    {
      reelMotor.set(-0.2);
    }
    else if(!bToggleState)
    {
      reelMotor.stopMotor();
    }
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() 
  {
    gyro.calibrate();
    SmartDashboard.putNumber("Reel Revolutions", 9);
    SmartDashboard.putNumber("Slew Rate", 3.0);
    m_chooser.setDefaultOption("2 ball high shooter Ball 1 (left)", kCustomAuto4);
    m_chooser.addOption("2 ball low shooter 1 (left)", kCustomAuto5);
    m_chooser.addOption("2 ball high shooter Ball 2 (middle)", kDefaultAuto);
    m_chooser.addOption("2 ball low shooter 2 (middle)", kCustomAuto6);
    m_chooser.addOption("2 ball high shooter Ball 3 (wall)", kCustomAuto3);
    m_chooser.addOption("High Auto", kCustomAuto2);
    m_chooser.addOption("Low Auto", kCustomAuto);
    limelightchooser.setDefaultOption(left, left);
    limelightchooser.addOption(right, right);
    SmartDashboard.putData("Auto choices", m_chooser);
    SmartDashboard.putData("Targeting Turn Direction", limelightchooser);
    // new motorgroups as leader/follower since MotorControllerGroup was depricated 
    SparkBaseConfig l2Config = new SparkMaxConfig().follow(motorL1, /*invert*/ false);
    SparkBaseConfig r2Config = new SparkMaxConfig().follow(motorR1, /*invert*/ false);
    motorL2.configure(l2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorR2.configure(r2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // TODO: Need to fix driving this does not work
    // leftGroup = new MotorControllerGroup(motorL1,motorL2); 
    //rightGroup = new MotorControllerGroup(motorR1,motorR2);
    //leftGroup.setInverted(true);
    //rightGroup.setInverted(true);
    // driveRobot = new DifferentialDrive(leftGroup,rightGroup);
    driveRobot = new DifferentialDrive(motorL1::set,motorR2::set);
    ledStrip.set(getTeamColor());
    // Removed the voltage compensation stuff for 2025 code rewrite for Ag Fair
    //outMotor.enableVoltageCompensation(12.0);
    //reelMotor.enableVoltageCompensation(9.0);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() 
  {
    SmartDashboard.updateValues();
    SmartDashboard.putNumber("Ball Sensor", feederSensor.getValue());
    timePassed = Timer.getFPGATimestamp() - startTime;
    SmartDashboard.putNumber("Gyro Values", gyro.getAngle());
    SmartDashboard.putNumber("Reel Motor Revs", reelMotor.getEncoder().getPosition());
    if(SmartDashboard.getNumber("Slew Rate", 3) != oldSlew)
    {
      driveAccLimiter = new SlewRateLimiter(SmartDashboard.getNumber("Slew Rate", 3));
      oldSlew = SmartDashboard.getNumber("Slew Rate", 3);
      //MAKE SURE ONLY TO ADJUST VALUE WHEN STOPPED
    }

  }
  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    gyro.reset();
    // motorL1.setIdleMode(IdleMode.kBrake);
    motorL1.configure(
      new SparkMaxConfig().idleMode(IdleMode.kBrake),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    motorL2.configure(
      new SparkMaxConfig().idleMode(IdleMode.kBrake),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    motorR1.configure(
      new SparkMaxConfig().idleMode(IdleMode.kBrake),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    motorR2.configure(
      new SparkMaxConfig().idleMode(IdleMode.kBrake),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    reelMotor.configure(
      new SparkMaxConfig().idleMode(IdleMode.kBrake),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    m_autoSelected = m_chooser.getSelected();
   
    System.out.println("Auto selected: " + m_autoSelected);
    startTime = Timer.getFPGATimestamp(); // get the match start time
    mode = "Drive Forward";
    bToggleState = false;
    reelMotor.getEncoder().setPosition(SmartDashboard.getNumber("Reel Revolutions", 9));
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //TODO: Remove the auto stuff for the Ag Fair code so we don't accidentally harm anyone
    autoFeedRoutine();
    intakeReel();
    switch (m_autoSelected)
    {
      case kCustomAuto:
        // Put custom auto code here orrr low ball shooter
        if((timePassed > 0) && (timePassed < 1))
        {
          outMotor.set(0.55);
        }
        else if((timePassed > 1 ) && (timePassed < 3))
        {
          feedMotor.set(feedSpeed);
        }
        else 
        {
          outMotor.stopMotor();
          feedMotor.stopMotor();
        }
        if((timePassed > 3) && (timePassed < 5))
        {
          driveRobot.arcadeDrive(0, -0.6);
        }
        else
        {
          driveRobot.stopMotor();
        }
        break;
      case kDefaultAuto:
        //auto code section 2 orrr 2 high middle ball shooter code  
        switch(mode)
        {
          case "Drive Forward":
            driveRobot.arcadeDrive(0, 0.5);
            inMotor.set(inSpeed);
            if(preFeedSensor.getValue() >= 800)
            {
              driveRobot.stopMotor();
              inMotor.stopMotor();
              driveDuration = timePassed;
              mode = "Turn Around";
            }
            break;
          case "Turn Around":
            driveRobot.arcadeDrive(0.4, 0);
            if(gyro.getAngle() >= 159)
            {
              driveRobot.stopMotor();
              mode = "Drive Back";
              driveBackStart = timePassed;
            }
            break;
          case "Drive Back":
            if(timePassed - driveBackStart + 0.3 <= driveDuration)
            {
              driveRobot.arcadeDrive(0, 0.45);
            }
            else
            {
              driveRobot.stopMotor();
              mode = "Shoot";
              shootTime = timePassed;
            }
            break;
          case "Shoot":
            if(timePassed - shootTime <= 4)
            {
              outMotor.set(highSpeed);
              if(timePassed - shootTime > 1.3)
              {
                feedMotor.set(feedSpeed);  
              }
            }
            else
            {
              outMotor.stopMotor();
              feedMotor.stopMotor();
            }
            break;
        }
        break;
      case kCustomAuto2:
        // Put default auto code here orr high ball shooter code 
        if((timePassed > 0) && (timePassed < 1.7))
        {
          driveRobot.arcadeDrive(0, -0.3);
        }
        else if((timePassed > 1.7) && (timePassed < 2.7))
        {
          driveRobot.stopMotor();
          outMotor.set(highSpeed);
        }
        else if((timePassed > 2.7) && (timePassed < 3.7))
        {
          feedMotor.set(feedSpeed);
        }
        else if((timePassed >  4.5) && (timePassed < 5.9))
        {
          outMotor.stopMotor();
          feedMotor.stopMotor();
          driveRobot.arcadeDrive(0, -0.5);
        }
        else
        {
          driveRobot.stopMotor();
        }
        break;
      case kCustomAuto3:
      //auto code section 2 orrr 2 high ball shooter code ball 3 (Wall) 
        switch(mode)
        {
          case "Drive Forward":
            driveRobot.arcadeDrive(0, 0.35);
            inMotor.set(inSpeed);
            if(preFeedSensor.getValue() >= 800)
            {
              driveRobot.stopMotor();
              inMotor.stopMotor();
              driveDuration = timePassed;
              gyro.reset();
              mode = "Turn Around";
            }
            break;
          case "Turn Around":
          if(gyro.getAngle() <= 130)
          {
            driveRobot.arcadeDrive(0.45, 0);
            inMotor.set(inSpeed);
          }
          else if(gyro.getAngle() <= 181)
          {
            driveRobot.arcadeDrive(0.3, 0);
            inMotor.set(inSpeed);
          }
          else
          {
            driveRobot.stopMotor();
            mode = "Drive Back";
            driveBackStart = timePassed;
          }
          break;
          case "Drive Back":
            inMotor.set(inSpeed);
            if(timePassed - driveBackStart + 1 <= driveDuration)
            {
              driveRobot.arcadeDrive(0, 0.35);
            }
            else
            {
              driveRobot.stopMotor();
              mode = "Shoot";
              shootTime = timePassed;
            }
          break;
          case "Shoot":
            if(timePassed - shootTime <= 4)
            {
              inMotor.set(inSpeed);
              outMotor.set(highSpeed);
              if(timePassed - shootTime > 1.3)
              {
                feedMotor.set(feedSpeed);  
              }
            }
            else
            {
              outMotor.stopMotor();
              feedMotor.stopMotor();
              inMotor.stopMotor();
            }
          break;
        }
        break;
      case kCustomAuto4:
      //auto code section 2 orrr 2 high ball shooter code ball 1 (left) 
        switch(mode)
        {
          case "Drive Forward":
            if(gyro.getAngle() >= -5)
            {
              driveRobot.arcadeDrive(-0.3, 0);
            }
            else
            {
              driveRobot.arcadeDrive(0, 0.5);
              inMotor.set(inSpeed);
              if(preFeedSensor.getValue() >= 800)
              {
                driveRobot.stopMotor();
                inMotor.stopMotor();
                driveDuration = timePassed;
                mode = "Turn Around";
                gyro.reset();
              }
            }
            break;
          case "Turn Around":
            if(gyro.getAngle() < 130)
            {
              driveRobot.arcadeDrive(0.4, 0);
            }
            else if(gyro.getAngle() <= 167)
            {
              driveRobot.arcadeDrive(0.3, 0);
            }
            else
            {
              driveRobot.stopMotor();
              mode = "Drive Back";
              driveBackStart = timePassed;
            }
            break;
          case "Drive Back":
            if(timePassed - driveBackStart + 0.3 <= driveDuration)
            {
              driveRobot.arcadeDrive(0, 0.45);
            }
            else
            {
              driveRobot.stopMotor();
              mode = "Shoot";
              shootTime = timePassed;
            }
            break;
          case "Shoot":
            if(timePassed - shootTime <= 4)
            {
              outMotor.set(highSpeed);
              if(timePassed - shootTime > 1.3)
              {
                feedMotor.set(feedSpeed);  
              }
            }
            else
            {
              outMotor.stopMotor();
              feedMotor.stopMotor();
            }
            break;
        }
        break;
      case kCustomAuto5:
        //2 ball low shooter 1 left 
        switch(mode)
        {
          case "Drive Forward":
            driveRobot.arcadeDrive(0, 0.5);
            inMotor.set(inSpeed);
            if(preFeedSensor.getValue() >= 800)
            {
              driveRobot.stopMotor();
              inMotor.stopMotor();
              driveDuration = timePassed;
              mode = "Turn Around";
            }
            break;
          case "Turn Around":
            if(gyro.getAngle() <= 130)
            {
              driveRobot.arcadeDrive(0.4, 0);
            }
            else if(gyro.getAngle() <= 166)
            {
              driveRobot.arcadeDrive(0.3, 0);
            }
            else
            {
              driveRobot.stopMotor();
              mode = "Drive Back";
              driveBackStart = timePassed;
            }
          break;
          case "Drive Back":
            if(timePassed - driveBackStart - 0.1 <= driveDuration)
            {
              driveRobot.arcadeDrive(0, 0.45);
            }
            else
            {
              driveRobot.stopMotor();
              mode = "Shoot";
              shootTime = timePassed;
            }
          break;
          case "Shoot":
            if(timePassed - shootTime <= 4)
            {
              outMotor.set(0.5);
              if(timePassed - shootTime > 1.3)
              {
                feedMotor.set(feedSpeed);  
              }
            }
            else
            {
              outMotor.stopMotor();
              feedMotor.stopMotor();
            }
          break;
        }
        break;
      case kCustomAuto6:
        // 2 ball low shooter for #2 middle
        switch(mode)
        {
          case "Drive Forward":
            driveRobot.arcadeDrive(0, 0.5);
            inMotor.set(inSpeed);
            if(preFeedSensor.getValue() >= 800)
            {
              driveRobot.stopMotor();
              inMotor.stopMotor();
              driveDuration = timePassed;
              mode = "Turn Around";
            }
            break;
          case "Turn Around":
            if(gyro.getAngle() <= 130)
            {
              driveRobot.arcadeDrive(0.4, 0);
            }
            else if(gyro.getAngle() <= 176)
            {
              driveRobot.arcadeDrive(0.3, 0);
            }
            else
            {
              driveRobot.stopMotor();
              mode = "Drive Back";
              driveBackStart = timePassed;
            }
            break;
          case "Drive Back":
            if(timePassed - driveBackStart - 0.4 <= driveDuration)
            {
              driveRobot.arcadeDrive(0, 0.45);
            }
            else
            {
              driveRobot.stopMotor();
              mode = "Shoot";
              gyro.reset();
            }
            break;
          case "Shoot":
            if(gyro.getAngle() >= -10)
            {
              driveRobot.arcadeDrive(-0.3, 0);
              shootTime = timePassed;
            }
            else
            {
              if(timePassed - shootTime <= 4)
              {
                outMotor.set(0.5);
                if(timePassed - shootTime > 1.3)
                {
                  feedMotor.set(feedSpeed);  
                }
              }
              else
              {
                outMotor.stopMotor();
                feedMotor.stopMotor();
              }
            }
            break;
        }
        break;
      default:
        break;
    }
  }
    

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    gyro.reset();
    // Set motors to brake mode
    motorL1.configure(
      new SparkMaxConfig().idleMode(IdleMode.kBrake),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    motorL2.configure(
      new SparkMaxConfig().idleMode(IdleMode.kBrake),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    motorR1.configure(
      new SparkMaxConfig().idleMode(IdleMode.kBrake),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    motorR2.configure(
      new SparkMaxConfig().idleMode(IdleMode.kBrake),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    reelMotor.configure(
      new SparkMaxConfig().idleMode(IdleMode.kBrake),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }
//#endregion
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
    ll_TurnDirection = limelightchooser.getSelected();
    Update_Limelight_Tracking();

    boolean auto = xBox.getRightTriggerAxis() > 0.2;

    if(xBox.getBButtonPressed())
    {
      bToggleState = !bToggleState;
    }
    reelCoast = xBox.getStartButton();
    startReleased = xBox.getStartButtonReleased();
    intakeReel();
    if(feederSensor.getValue() > 300 && preFeedSensor.getValue() > 500)
    {
      ledStrip.set(green);
    }
    else
    {
      ledStrip.set(getTeamColor());
    }

    //double motorSpeed = xBox.getLeftY() * inputScaling;
    //SmartDashboard.putNumber("motorSpeed", motorSpeed);
    SmartDashboard.putNumber("Speed of Drivetrain", inputScaling);
    if(xBox.getLeftTriggerAxis() >= 0.99)
    {
      driveRobot.stopMotor();
    }
    else if(auto)
    {
      if (m_LimelightHasValidTarget)
      {
        driveRobot.arcadeDrive(m_LimelightSteerCommand,m_LimelightDriveCommand);
      }
      else
      {
        driveRobot.arcadeDrive((ll_TurnDirection == "left" ? -0.5 : 0.5),0.0);
      }
    }
    else
    {
      driveRobot.arcadeDrive((xBox.getLeftX() * inputScaling),driveAccLimiter.calculate(-xBox.getLeftY() * inputScaling));
    }
    if(povState != xBox.getPOV()) 
    {
      povState = xBox.getPOV();
      if(povState == 180) 
      { 
        inputScaling = 0.4;
      }
      else if(povState == 0)
      {
        inputScaling = 0.7;
      }
      else if(povState == 270)
      { 
        inputScaling = 0.55;
      }
      else if(povState == 90)
      {
        inputScaling = 1.0;
      }
    }
        
    driveRobot.setMaxOutput(1.0 - xBox.getLeftTriggerAxis());

    //Intake motor intake toggle
    if(xBox.getAButton() || joystick.getRawButton(3))
    {
      aToggleState = true;
    }
    else
    {
      aToggleState = false;
    }
    
    //Intake motor sensor toggle off if both sensors detect ball as there will be 2 balls.
    if(feederSensor.getValue() > 300 && preFeedSensor.getValue() > 800)
    {
      aToggleState = false;
    }

    //Intake motor speed set
    if(aToggleState || joystick.getRawButton(3))
    {
      //inMotor.set(0.3);
      inMotor.set(inSpeed);
      //SmartDashboard.putString("Abutton", "pushed");
    }
    else if(!xBox.getXButton())
    {
      inMotor.stopMotor();
      //SmartDashboard.putString("Abutton", "not pushed");
    }
    
    //timePassed = Timer.getFPGATimestamp() - startTime; 

    /*if(xBox.getLeftBumperPressed())
    {
      speedIndex++;
      speedIndex = speedIndex % outSpeeds.length;
    }*/
    if(xBox.getLeftBumperButtonPressed() || joystick.getRawButtonPressed(1))
    {
      lowFeedStart = Timer.getFPGATimestamp();
    }

    if(xBox.getRightBumperButtonPressed() || joystick.getRawButton(2))
    {
      highFeedStart = Timer.getFPGATimestamp();
    }
    
    if(xBox.getLeftBumperButton() || joystick.getRawButton(1))
    {

      outMotor.set(lowSpeed);
      if(Timer.getFPGATimestamp() - lowFeedStart > 0.5)
      {
        feedMotor.set(feedSpeed);
      }
      else if(!xBox.getXButton() && !xBox.getYButton())
      {
        feedMotor.stopMotor();
      }
    }
    else if(xBox.getRightBumperButton() || joystick.getRawButton(2))
    {
      outMotor.set(highSpeed);
      if(Timer.getFPGATimestamp() - highFeedStart > 1)
      {
        feedMotor.set(feedSpeed);
      }
      else if(!xBox.getXButton() && !xBox.getYButton())
      {
        feedMotor.stopMotor();
      }
    }
    else
    {
      outMotor.stopMotor();
    }
    
    if(xBox.getYButton())
    {
      feedMotor.set(feedSpeed);
      //SmartDashboard.putNumber("feedSpeed", feedSpeed);
    }
    else if(!(xBox.getLeftBumperButton() || xBox.getRightBumperButton() || xBox.getXButton()))
    {
      feedMotor.stopMotor();
      //SmartDashboard.putNumber("feedSpeed", 0);
    }

    if((xBox.getXButton() || joystick.getRawButton(4)) && !(xBox.getLeftBumperButton() || xBox.getRightBumperButton() || xBox.getYButton()))
    {
      feedMotor.set(-1.5 * feedSpeed);
      //SmartDashboard.putNumber("feedSpeed", -feedSpeed);
      inMotor.set(-inSpeed);
      //SmartDashboard.putNumber("inSpeed", -inSpeed);
      aToggleState = false;
    }
    else if(!xBox.getLeftBumperButton() && !(xBox.getLeftBumperButton() || xBox.getRightBumperButton() || xBox.getYButton()))
    {
      feedMotor.stopMotor();
      //SmartDashboard.putNumber("feedSpeed", 0); 
      if(!aToggleState)
      {
        inMotor.stopMotor();
        //SmartDashboard.putNumber("inSpeed", 0);
      }
    }

    if(xBox.getXButtonReleased())
    {
      reverseDelay = Timer.getFPGATimestamp();
    }
    
    //If a ball is detected at the upper sensor or the lower sensor is clear and the timer was running for more than 3 seconds
    autoFeedRoutine();

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit()
  {
    // Set the motors to coast for easy robot manipulaton
    motorL1.configure(
      new SparkMaxConfig().idleMode(IdleMode.kCoast),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    motorL2.configure(
      new SparkMaxConfig().idleMode(IdleMode.kCoast),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    motorR1.configure(
      new SparkMaxConfig().idleMode(IdleMode.kCoast),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    motorR2.configure(
      new SparkMaxConfig().idleMode(IdleMode.kCoast),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    reelMotor.configure(
      new SparkMaxConfig().idleMode(IdleMode.kCoast),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
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
