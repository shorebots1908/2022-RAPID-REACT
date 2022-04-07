// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* TODOs
* Add reverse to feed mechanism -
* Add reverse to intake mechanism -
* Implement sensor stop to feed mechanism
* Implement sensor stop to intake mechanism
* Ultrasonic for shooting range
* Color sensor output to LED lightstrips
* change input scaling to 1, 0.7, 0.3 on up, right,down d-pad. -
* LED flash when ball grabbed
* LED signal when balls full
* LED team colors
* Add a timer for reverse and sensor activation
* Gyroscope Code
* Auto code
* LED signal when having one ball - flash green once 
* LED Signal when having one ball - constant flashing green
*/

package frc.robot;
//import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import javax.management.relation.Relation;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.cameraserver.CameraServer;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.filter.SlewRateLimiter;
public class Robot extends TimedRobot {
  private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  private UsbCamera camera1;
  private UsbCamera camera2;
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
  private Joystick joystick = new Joystick(0);
  //private final SendableChooser<String> reelChooser = new SendableChooser<>();
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  //private Joystick stickL = new Joystick(0);
  //private Joystick stickR = new Joystick(1);
  private CANSparkMax motorL1 = new CANSparkMax(10, MotorType.kBrushless);
  private CANSparkMax motorL2 = new CANSparkMax(11, MotorType.kBrushless);
  private CANSparkMax motorR1 = new CANSparkMax(12, MotorType.kBrushless);
  private CANSparkMax motorR2 = new CANSparkMax(13, MotorType.kBrushless);
  private CANSparkMax inMotor = new CANSparkMax(7, MotorType.kBrushless);
  private CANSparkMax outMotor = new CANSparkMax(9, MotorType.kBrushless);
  private CANSparkMax feedMotor = new CANSparkMax(8, MotorType.kBrushless);
  private CANSparkMax reelMotor = new CANSparkMax(6, MotorType.kBrushless);
  private boolean reelCoast = false;
  private boolean startReleased = false;
  private DifferentialDrive driveRobot;
  private MotorControllerGroup leftGroup;
  private MotorControllerGroup rightGroup;
  //private final I2C.Port i2cPort = I2C.Port.kOnboard;
  //private ColorSensorV3 ColorSensor = new ColorSensorV3(i2cPort);
  private AnalogInput feederSensor = new AnalogInput(2);
  private AnalogInput preFeedSensor = new AnalogInput(3);
//   private AnalogInput distanceSensor = new AnalogInput(2);
  private SlewRateLimiter driveAccLimiter = new SlewRateLimiter(3);
  private double heading;
  private Spark ledStrip = new Spark(0);
  private double green = 0.71;
  //private double teamColor;
  private boolean isRedAlliance = false;
  NetworkTable FMS = NetworkTableInstance.getDefault().getTable("FMSInfo");
  //auto variable programs
  //control variables
  private double inputScaling = 0.4;
  private int povState = -1;
  //private int speedIndex = 0;
  private boolean aToggleState = false, bToggleState = false;
  private double timePassed;
  private double driveDuration, driveBackStart, shootTime;
  private double highFeedStart, lowFeedStart;
  private boolean feedFlag = false;
  private String mode;
  private double reverseDelay;
  private double autoFeedTimeStart;
  //private int backTime = 3;
  //private boolean findBall = false, isBallFound = false;
  //private double initialRange;
  //configuration variables
  private double inSpeed = -0.7;
  //private double outSpeeds[] = {0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
  private double feedSpeed = 0.6;
  private double highSpeed = 0.9;
  private double lowSpeed = 0.45; 
  //Custom Functions
  /*
  private double ultraInches(double _raw)
  {
    double voltage_scale_factor = 5/RobotController.getVoltage5V();
    return _raw * voltage_scale_factor * 0.0492;
  }
  */

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

    if(feedFlag && !(xBox.getLeftBumper() || xBox.getRightBumper() || xBox.getXButton() || xBox.getYButton()))
    {
      feedMotor.set(0.3);
    } 
    else if(!feedFlag && !(xBox.getLeftBumper() || xBox.getRightBumper() || xBox.getXButton() || xBox.getYButton()))
    {
      feedMotor.stopMotor();
    }
  }

  private void intakeReel()
  {
    double reelRevs = SmartDashboard.getNumber("Reel Revolutions", 8.0);
    double reelPosition = reelMotor.getEncoder().getPosition();
    if(reelCoast)
    {
     reelMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }
    else if(startReleased)
    {
    reelMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
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
    camera1 = CameraServer.startAutomaticCapture(0);
    camera2 = CameraServer.startAutomaticCapture(1);
    SmartDashboard.putNumber("Reel Revolutions", 8.0);
    m_chooser.setDefaultOption("2 ball high shooter Ball 1 (left)", kCustomAuto4);
    m_chooser.addOption("2 ball low shooter 1 (left)", kCustomAuto5);
    m_chooser.addOption("2 ball high shooter Ball 2 (middle)", kDefaultAuto);
    m_chooser.addOption("2 ball low shooter 2 (middle)", kCustomAuto6);
    m_chooser.addOption("2 ball high shooter Ball 3 (wall)", kCustomAuto3);
    m_chooser.addOption("High Auto", kCustomAuto2);
    m_chooser.addOption("Low Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    //motorR2.setInverted(true);
    leftGroup = new MotorControllerGroup(motorL1,motorL2);
    rightGroup = new MotorControllerGroup(motorR1,motorR2);
    //leftGroup.setInverted(true);
    //rightGroup.setInverted(true);
    driveRobot = new DifferentialDrive(leftGroup,rightGroup);
    /*SmartDashboard.putString("a Button", "run intake motor");
    SmartDashboard.putString("x Button", "reverse intake motor");
    SmartDashboard.putString("y Button", "run feed motor");
    //SmartDashboard.putString("bButton", "emergerencyBrake");
    SmartDashboard.putString("up d-pad", "full speed");
    SmartDashboard.putString("down d-pad", "slow speed");
    SmartDashboard.putString("left d-pad", "middle speed");
    SmartDashboard.putString("left trigger", "brake throttle");
    SmartDashboard.putString("left bumper", "low shoot");
    SmartDashboard.putString("right bumper", "high shoot");
 */ ledStrip.set(getTeamColor());
    outMotor.enableVoltageCompensation(12.0);
    reelMotor.enableVoltageCompensation(9.0);
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
    SmartDashboard.putNumber("Ball Sensor", feederSensor.getValue());
    //SmartDashboard.putNumber("distanceSensorValue", ultraInches(distanceSensor.getValue()));
    //SmartDashboard.putNumber("Throwing Speed", outSpeeds[speedIndex]);
    //SmartDashboard.putNumber("preSensor", preFeedSensor.getValue());
    //SmartDashboard.putNumber("IR Input", feederSensor.getValue());
    timePassed = Timer.getFPGATimestamp() - startTime;
    //SmartDashboard.putBoolean("Ballfinding", bToggleState);
    SmartDashboard.putNumber("Gyro Values", gyro.getAngle());
    SmartDashboard.putNumber("Reel Motor Revs", reelMotor.getEncoder().getPosition());
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
    motorL1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    motorL2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    motorR1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    motorR2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    reelMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_autoSelected = m_chooser.getSelected();
    //m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    startTime = Timer.getFPGATimestamp(); // get the match start time
    mode = "Drive Forward";
    bToggleState = false;
    reelMotor.getEncoder().setPosition(SmartDashboard.getNumber("Reel Revolutions", 9.0));
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
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
    motorL1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    motorL2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    motorR1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    motorR2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    reelMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

  }
//#endregion
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
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

    double motorSpeed = xBox.getLeftY() * inputScaling;
    //SmartDashboard.putNumber("motorSpeed", motorSpeed);
    SmartDashboard.putNumber("Speed of Drivetrain", inputScaling);
    if(xBox.getLeftTriggerAxis() >= 0.99)
    {
      driveRobot.stopMotor();
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
        inputScaling = 0.65;
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

    //Toggle ball finding mode
    /*
    if(xBox.getBButtonPressed())
    {
      bToggleState = !bToggleState;
      initialRange = ultraInches(distanceSensor.getValue());
    }

    
    if(bToggleState)
    {
      if(ultraInches(distanceSensor.getValue()) <= initialRange -3)
      {
        bToggleState = false;
        driveRobot.arcadeDrive(0, 0);
      }
      else
      {
        driveRobot.arcadeDrive(-0.2, 0);
      }
    }
    */

    //Intake motor intake toggle
    if(xBox.getAButton())
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
    if(aToggleState)
    {
      //inMotor.set(0.3);
      inMotor.set(inSpeed);
      //SmartDashboard.putString("Abutton", "pushed");
    }
    else if(!xBox.getXButton())
    {
      //inMotor.stopMotor();
      inMotor.stopMotor();
      //SmartDashboard.putString("Abutton", "not pushed");
    }
    
    //timePassed = Timer.getFPGATimestamp() - startTime; 

    /*if(xBox.getLeftBumperPressed())
    {
      speedIndex++;
      speedIndex = speedIndex % outSpeeds.length;
    }*/
    if(xBox.getLeftBumperPressed())
    {
      lowFeedStart = Timer.getFPGATimestamp();
    }

    if(xBox.getRightBumperPressed())
    {
      highFeedStart = Timer.getFPGATimestamp();
    }
    
    if(xBox.getLeftBumper())
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
    else if(xBox.getRightBumper())
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
    else if(!(xBox.getLeftBumper() || xBox.getRightBumper() || xBox.getXButton()))
    {
      feedMotor.stopMotor();
      //SmartDashboard.putNumber("feedSpeed", 0);
    }

    if(xBox.getXButton() && !(xBox.getLeftBumper() || xBox.getRightBumper() || xBox.getYButton()))
    {
      feedMotor.set(-1.5 * feedSpeed);
      //SmartDashboard.putNumber("feedSpeed", -feedSpeed);
      inMotor.set(-inSpeed);
      //SmartDashboard.putNumber("inSpeed", -inSpeed);
      aToggleState = false;
    }
    else if(!xBox.getLeftBumper() && !(xBox.getLeftBumper() || xBox.getRightBumper() || xBox.getYButton()))
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
    
    // double voltage_scale_factor = 5/RobotController.getVoltage5V();
    // double currentDistanceInches = distanceSensor.getValue() * voltage_scale_factor * 0.0492;
    //SmartDashboard.putNumber("Distance Sensor Inches", currentDistanceInches);
    
    //If a ball is detected at the upper sensor or the lower sensor is clear and the timer was running for more than 3 seconds
    autoFeedRoutine();

      //if(feederSensor.getValue()>= 300)
    /*if(!(feederSensor.getValue() >=300) && !(xBox.getYButton()) && !(xBox.getXButton() && !(xBox.getRightBumper())))
    {
      if(preFeedSensor.getValue() >= 800)
      {
        feedMotor.set(0.3);
      }
      else if((feederSensor.getValue() >=300))
      {
        feedMotor.stopMotor();
        
      }
    }*/
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit()
  {
    motorL1.setIdleMode(CANSparkMax.IdleMode.kCoast);
    motorL2.setIdleMode(CANSparkMax.IdleMode.kCoast);
    motorR1.setIdleMode(CANSparkMax.IdleMode.kCoast);
    motorR2.setIdleMode(CANSparkMax.IdleMode.kCoast);
    reelMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() 
  {
    reelMotor.setIdleMode(IdleMode.kBrake);
    reelMotor.enableVoltageCompensation(6.0);
    reelMotor.getEncoder().setPosition(SmartDashboard.getNumber("Reel Revolutions", 9.0));
    bToggleState = false;
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic()
  {
    SmartDashboard.putNumber("Reel Motor Revs", reelMotor.getEncoder().getPosition());
    if(xBox.getBButtonPressed())
    {
      bToggleState = !bToggleState;
    }
    intakeReel();
  }
}
