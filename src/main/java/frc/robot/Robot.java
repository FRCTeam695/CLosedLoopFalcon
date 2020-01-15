/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;




class Gains {
	public final double kP;
	public final double kI;
	public final double kD;
	public final double kF;
	public final int kIzone;
	public final double kPeakOutput;
	
	public Gains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput){
		kP = _kP;
		kI = _kI;
		kD = _kD;
		kF = _kF;
		kIzone = _kIzone;
		kPeakOutput = _kPeakOutput;
	}
}




class Constants {
	/**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
    public static final int kTimeoutMs = 30;

	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 7200 represents Velocity units at 100% output
     * 
	 * 	                                    			          kP   kI   kD   kF          Iz    PeakOut */
    public final static Gains kGains_Velocit = new Gains( 0.23, 0.0004, 7, 0,  300,  1.00);
}















/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private TalonFX mytalon = new TalonFX(11);
  private Joystick toggler = new Joystick(0);

  //private Joystick m_stick = new Joystick(0);


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    System.out.println("695:  robotInit()");

    /* Factory Default all hardware to prevent unexpected behaviour */
    mytalon.configFactoryDefault();

    /* Config sensor used for Primary PID [Velocity] */
    mytalon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor ,
                                          Constants.kPIDLoopIdx, 
                                          Constants.kTimeoutMs);

    /**
    * Phase sensor accordingly. 
    * Positive Sensor Reading should match Green (blinking) Leds on Talon
    */
    mytalon.setSensorPhase(true);

    /* Config the peak and nominal outputs */
    mytalon.configNominalOutputForward(0, Constants.kTimeoutMs);
    mytalon.configNominalOutputReverse(0, Constants.kTimeoutMs);
    mytalon.configPeakOutputForward(1, Constants.kTimeoutMs);
    mytalon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /* Config the Velocity closed loop gains in slot0 */
    mytalon.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
    mytalon.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
    mytalon.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
    mytalon.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);


  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    System.out.println("695:  disabledInit()");
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    System.out.println("695:  autonomousInit()");
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
//    mytalon.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void teleopInit() {
    System.out.println("695:  teleopInit()");
  }

  /**
   * This function is called periodically during operator control.
   */
  private double RPMVelocity = 3000;
  private boolean povDebounce = false;
  private double lastPOV = 0;
  public void changeRPM(double delta) {
    RPMVelocity += delta;
    if (Math.abs(RPMVelocity) > 6000) {
      RPMVelocity = 6000*(Math.abs(RPMVelocity)/RPMVelocity);
    }
  }

  @Override
  public void teleopPeriodic() {
//    double targetVelocity_UnitsPer100ms = leftYstick * 500.0 * 4096 / 600;
/*
    double targetVelocity_UnitsPer100ms = 60 * 4096 / 600;
    mytalon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
    */

    if (toggler.getRawButton(1)) {
      double targetVelocity_UnitsPer100ms = RPMVelocity * 2048 / 600;
      /* 500 RPM in either direction */
      mytalon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
    } else {
      mytalon.set(ControlMode.PercentOutput, 0);

    }

    if (!povDebounce) {
      if (toggler.getPOV() == 0) {
        changeRPM(500);
      } else if(toggler.getPOV() == 180) {
        changeRPM(-500);
      }
    }
    if (lastPOV != toggler.getPOV()) {
      povDebounce = false;
    } else {
      povDebounce = true;
    }
    lastPOV = toggler.getPOV();
    
    
    //mytalon.set(ControlMode.PercentOutput, 0.1);

    double myRPM = mytalon.getSelectedSensorVelocity(0) * 600 / 2048;
    System.out.println(myRPM);

  }

  @Override
  public void testInit() {
    System.out.println("testInit()");
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
