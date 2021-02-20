
package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private XboxController m_driverController;
  private XboxController m_copilotController;


  // Drive Base
  private WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(5);
  private WPI_TalonSRX m_rearLeft = new WPI_TalonSRX(6);
  private  WPI_TalonSRX m_frontRight = new WPI_TalonSRX(1);
  private  WPI_TalonSRX m_rearRight = new WPI_TalonSRX(8);

  // Intake
  private  WPI_TalonSRX m_intake = new WPI_TalonSRX(2);

  // Spinner
  private WPI_TalonSRX m_spinner = new WPI_TalonSRX(3);

  //Wheel to load ball

  private WPI_TalonSRX m_wheel = new WPI_TalonSRX(4);

  // Shooter
  private CANSparkMax m_leftShooter = new CANSparkMax(12, MotorType.kBrushless);
  private CANSparkMax m_rightShooter = new CANSparkMax(9, MotorType.kBrushless);
  private CANSparkMax m_turretMotor = new CANSparkMax(10, MotorType.kBrushless);
  private boolean wheelPrev = false; 
  private boolean wheelState = false;

  private LimeLight camera;


 private Compressor c = new Compressor(0);
  private boolean intakeSwitchPrev = false;
  private boolean intakeState = true;
  private Solenoid intake_in = new Solenoid(2);
  private Solenoid intake_out = new Solenoid(3);
  private boolean motorSwitchPrev = false;
  private boolean motorState = false;

  // Kauail Labs AHRS (for heading and rate)
  //private AHRS ahrs = new AHRS(SPI.Port.kMXP);

  @Override
  public void robotInit() {
    m_driverController = new XboxController(0);
    m_copilotController = new XboxController(1);


    m_rearLeft.setInverted(true);
    m_rearRight.setInverted(true);
    m_frontLeft.setInverted(true);
    m_frontRight.setInverted(true);


    // Defaults to drift on neutral, this sets brake mode
    //m_rearLeft.setNeutralMode(NeutralMode.Brake);
    //m_rearRight.setNeutralMode(NeutralMode.Brake);
    //m_frontLeft.setNeutralMode(NeutralMode.Brake);
    //m_frontRight.setNeutralMode(NeutralMode.Brake);


    m_rearLeft.follow(m_frontLeft);
    m_rearRight.follow(m_frontRight);

    m_rightShooter.follow(m_leftShooter, true);

    c.setClosedLoopControl(true);
  }

  @Override
  public void teleopPeriodic() {
    //m_myRobot.tankDrive(m_driverController.getY(Hand.kLeft), m_driverController.getY(Hand.kRight));
    
    double reverse = m_driverController.getTriggerAxis(Hand.kLeft);
    double forward = m_driverController.getTriggerAxis(Hand.kRight);
    double front_back = reverse < 0.1 ? forward : -reverse;
    double left_right = m_driverController.getX(Hand.kLeft);


    left_right = left_right > 0 ? 0.5*Math.pow(Math.abs(left_right), 3) : -0.5*Math.pow(Math.abs(left_right), 3);
    

    double left = -front_back - left_right;
    double right = front_back - left_right;

    if(Math.abs(left) < 0.1)
      left = 0;
    if(Math.abs(right) < 0.1)
      right = 0;

    m_frontRight.set(right);
    m_frontLeft.set(left);

    if (m_copilotController.getBumperPressed(Hand.kLeft)==true && wheelPrev==false)
    {

      wheelState = !wheelState;
    }
    
    wheelPrev = m_copilotController.getBumperPressed(Hand.kLeft);
    
    if(wheelState == true)
    {
      m_leftShooter.set(-0.5);
    }
    else
    {
      m_leftShooter.set(0);
    }
    
    //m_rightShooter.set(1);
    
    //m_rightShooter.set(0);

    if(m_copilotController.getYButton())
    {
      m_intake.set(-1);
    }
    else 
    {
      m_intake.set(0);
    }

    if(m_copilotController.getXButton()==true && intakeSwitchPrev == false)
    {
        intakeState = !intakeState;
    }
    intakeSwitchPrev = m_copilotController.getXButton();
    intake_in.set(intakeState);
    intake_out.set(!intakeState);

    //when B is pressed inner motor spins 
    if(m_copilotController.getBButton()==true && motorSwitchPrev == false)
    {
        motorState = !motorState;
    }
    motorSwitchPrev = m_copilotController.getBButton();
    if(motorState)
    {
      m_spinner.set(-1);
    }
    else
    {
      m_spinner.set(0);
    }

    if(m_copilotController.getAButton())
    {
      m_wheel.set(-1);
    }
    else
    {
      m_wheel.set(0);
    }

    double turretSpeed = m_copilotController.getX(Hand.kRight);

    if(Math.abs(turretSpeed) > 0.1)
    {
      m_turretMotor.set(turretSpeed/5);
      System.out.println("Manual Mode");
    }
    else
    {
        boolean camTarget = camera.isTarget();
        double camSpeed;
        if(camTarget == true)
        {
          camSpeed = camera.getTx() * 0.01;
          System.out.println("Camspeed: " + camSpeed);
        }
        else
        {
          System.out.println("No target detected");
          camSpeed = 0;
        }
         
        if(camSpeed > .2)
        {
          camSpeed = .2;
        }
        else if(camSpeed < -.2)
        {
          camSpeed = -.2;
        }
        m_turretMotor.set(-camSpeed);
    }
    
     
     
  }


  }
 