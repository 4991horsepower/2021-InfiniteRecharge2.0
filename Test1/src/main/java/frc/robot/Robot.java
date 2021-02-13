
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
  private XboxController m_gamePad;

  // Drive Base
  private WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(5);
  private WPI_TalonSRX m_rearLeft = new WPI_TalonSRX(6);
  private  WPI_TalonSRX m_frontRight = new WPI_TalonSRX(1);
  private  WPI_TalonSRX m_rearRight = new WPI_TalonSRX(8);

  // Intake
  private  WPI_TalonSRX m_intake = new WPI_TalonSRX(2);

  // Spinner
  private WPI_TalonSRX m_spinner = new WPI_TalonSRX(3);

  // Shooter
  private CANSparkMax m_leftShooter = new CANSparkMax(12, MotorType.kBrushless);
  private CANSparkMax m_rightShooter = new CANSparkMax(9, MotorType.kBrushless);


  //private LimeLight camera;
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
    m_gamePad = new XboxController(0);

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

    c.setClosedLoopControl(true);
  }

  @Override
  public void teleopPeriodic() {
    //m_myRobot.tankDrive(m_gamePad.getY(Hand.kLeft), m_gamePad.getY(Hand.kRight));
    
    double reverse = m_gamePad.getTriggerAxis(Hand.kLeft);
    double forward = m_gamePad.getTriggerAxis(Hand.kRight);
    double front_back = reverse < 0.1 ? forward : -reverse;
    double left_right = m_gamePad.getX(Hand.kLeft);


    left_right = left_right > 0 ? 0.5*Math.pow(Math.abs(left_right), 3) : -0.5*Math.pow(Math.abs(left_right), 3);
    

    double left = -front_back - left_right;
    double right = front_back - left_right;

    if(Math.abs(left) < 0.1)
      left = 0;
    if(Math.abs(right) < 0.1)
      right = 0;

    m_frontRight.set(right);
    m_frontLeft.set(left);

    if(m_gamePad.getYButton())
    {
      m_intake.set(-1);
    }
    else 
    {
      m_intake.set(0);
    }

    if(m_gamePad.getXButton()==true && intakeSwitchPrev == false)
    {
        intakeState = !intakeState;
    }
    intakeSwitchPrev = m_gamePad.getXButton();
    intake_in.set(intakeState);
    intake_out.set(!intakeState);

    //when B is pressed inner motor spins 
    if(m_gamePad.getBButton()==true && motorSwitchPrev == false)
    {
        motorState = !motorState;
    }
    motorSwitchPrev = m_gamePad.getBButton();
    if(motorState)
    {
      m_spinner.set(-1);
    }
    else
    {
      m_spinner.set(0);
    }
  }


  }
 