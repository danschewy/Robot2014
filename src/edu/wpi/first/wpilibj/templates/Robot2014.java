 /*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.AnalogModule;
import edu.wpi.first.wpilibj.Dashboard;
import edu.wpi.first.wpilibj.DigitalModule;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Victor;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot2014 extends IterativeRobot {
    /* constants */
    private static final int JOYSTICK_USB_PORT = 1;
	private static final int ARM_CONTROL_AXIS = 3; 
	private static final double ARM_FULL_SPEED = 1d;
	private static final double ARM_REVERSE_SPEED = -.2d;
	private static final double ARM_NO_SPEED = 0d;
	private static final double ARM_TIMEOUT_THRESHOLD = .5d;
	private static final int LEFT_DRIVE_SPEEDCONTROLLER_PWM_PORT = 1;
	private static final int RIGHT_DRIVE_SPEEDCONTROLLER_PWM_PORT = 6;
	private static final int PHOTO_SENSOR_D_I_O_PORT = 1;
	private static final int ROBOT_MOVEMENT_AXIS = 2;
	private static final int ROBOT_ROTATE_AXIS = 1;
	private static final boolean SQUARED_INPUTS = true;
	private static final double SCOOP_UP_SPEED = .65d;
	private static final double SCOOP_DOWN_SPEED = -0.25d;
	private static final double NOT_SCOOPING = 0d;
	private static final int TICK_START = 10;
	private static final int ROTATION_INVERSION_D_S_D_I = 1;
	/* member variables */
    private GenericHID joystick;
    private DriverStationLCD driverStationLCD;
    private DriverStation driverStation;
	private SpeedController armSpeedController;
	private SpeedController scoopSpeedController;
	private SpeedController leftDriveSpeedController;
	private SpeedController rightDriveSpeedController;
	private Boolean armState;  // null = no spin; true = forward; false = reverse
	private Timer armTimer;
	private DigitalInput photoSensorDigitalInput;
	private RobotDrive robotDrive;
	private int scoopTicks=0;
	private boolean invertDriveRotation = true;
	/**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        this.joystick = new Joystick(JOYSTICK_USB_PORT);
        this.driverStationLCD = DriverStationLCD.getInstance();
		this.driverStation = DriverStation.getInstance();
		this.armSpeedController = new Victor(3);
		this.scoopSpeedController = new Talon(4);
		this.armTimer = new Timer();
		this.photoSensorDigitalInput = new DigitalInput(PHOTO_SENSOR_D_I_O_PORT);
		this.leftDriveSpeedController = new Talon(LEFT_DRIVE_SPEEDCONTROLLER_PWM_PORT);
		this.rightDriveSpeedController = new Talon(RIGHT_DRIVE_SPEEDCONTROLLER_PWM_PORT);
		this.robotDrive = new RobotDrive(this.leftDriveSpeedController, this.rightDriveSpeedController);
		
	}

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

    }
	
	 
	/** 
	 * This function is called once you hit enable for periodic mode
	 */
	public void teleopInit() {
		this.driverStationLCD.clear();
		this.invertDriveRotation = this.driverStation.getDigitalIn(ROTATION_INVERSION_D_S_D_I);
		this.driverStationLCD.println(DriverStationLCD.Line.kUser1, 1, (this.invertDriveRotation)?"Inverted Rotation    ":"Not inverted rotation");
		this.driverStationLCD.updateLCD();
	}
    
	public void teleopDrive(){
		double moveValue = this.joystick.getRawAxis(ROBOT_MOVEMENT_AXIS);
		double rotateValue = this.joystick.getRawAxis(ROBOT_ROTATE_AXIS);
		this.robotDrive.arcadeDrive( moveValue, ((this.invertDriveRotation)?-1d:1d)*rotateValue, SQUARED_INPUTS);
		
	} 
	
	
	
	/**
     * This function is called periodically during operator control
     */
	public void teleopPeriodic() {
		this.teleopScooper();
		this.teleopArm();
		this.teleopDrive();
		this.driverStationLCD.updateLCD();    
		
		this.updateDashboard();
	
	}
    
	/** 
	 * This function is called once you hit enable for periodic mode
	 */
	public void testInit() {
		this.driverStationLCD.clear();
	}
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
		for (int x=1;x<13;x++){
			if (this.joystick.getRawButton(x)) {
				this.driverStationLCD.println(DriverStationLCD.Line.kUser1, x, "1");
			} else {
				this.driverStationLCD.println(DriverStationLCD.Line.kUser1, x, "0");
			}
		}
		for (int axis=1;axis<7;axis++){		
			int pos = (axis%2==0)? 12 : 1;
			DriverStationLCD.Line line;	
			if (axis<3){
				line = DriverStationLCD.Line.kUser3;
			}
			else if (axis<5){
				line = DriverStationLCD.Line.kUser4;
			}
			else{
				line = DriverStationLCD.Line.kUser5;
			}
			
			double axisValue = this.joystick.getRawAxis(axis);

			StringBuffer axisString = new StringBuffer().append(axisValue); //converting the initial value which is double to string buffer

			if (Math.abs(axisValue)>axisValue){
				axisString.setLength(8);
			}
			else if (axisValue==0d || axisValue==1d){
				axisString.setLength(8);
			}
			else {
				axisString.setLength(7);
				axisString.setLength(8);
				//axisString = axis + ":" + axisString;
			}
			axisString.insert(0, axis + ":"); // starting from the beginnng it puts the axis and the colon to show what will be the numbers
			this.driverStationLCD.println(line, pos, axisString); //prints out the value into the LCD
			
			
/*			if (Math.abs(axisValue)>axisValue){
				StringBuffer axisString = new StringBuffer().append(axisValue);
				axisString.setLength(8);
				axisString.insert(0, axis + ":");
				System.out.println("negative axis:" + axisString);
				this.driverStationLCD.println(line, pos, axisString);
			}
			else if (axisValue==0d || axisValue==1d){
				System.out.println("whole number:" + axisValue);
				StringBuffer axisString = new StringBuffer().append(axisValue);
				axisString.setLength(8);
				axisString.insert(0, axis + ":");
				this.driverStationLCD.println(line, pos, axisString);
			}
			else {
				StringBuffer axisString = new StringBuffer().append(axisValue);
				axisString.setLength(7);
				axisString.setLength(8);
				//axisString = axis + ":" + axisString;
				axisString.insert(0, axis + ":");
				System.out.println("positive axis:" + axisString);
				this.driverStationLCD.println(line, pos, axisString);
			} */
		}
		this.driverStationLCD.updateLCD();    
	}
	private void teleopArm() {
	// the button -when axis 3=-1, motor will run only for 1 second
		String spinningString;
		switch ((this.armState.booleanValue())?1:0) {
			case 1:  // do I remember leaving it spinning forward?
				if (this.armTimer.get()>= ARM_TIMEOUT_THRESHOLD) {
					this.armSpeedController.set(ARM_NO_SPEED);
					this.armTimer.stop();
					this.armState = null;
					spinningString="Not Spinning      ";
				} else {
					this.armState = Boolean.TRUE;
					spinningString="Spinning Forward  ";
				}
				break;
			case 0:  // do I remember leaving it spinning backwards?
				if (this.photoSensorDigitalInput.get()) {
					this.armSpeedController.set(ARM_NO_SPEED);
					this.armState = null;
					spinningString="Not Spinning      ";
				}
				else {
					this.armState = Boolean.FALSE;
					spinningString="Spinning Backwards";
				}
				break;
			default:  // I remember not leaving it spinning at all
				double axisValue = this.joystick.getRawAxis(ARM_CONTROL_AXIS);
				if (axisValue == -1d) {
					this.armSpeedController.set(ARM_FULL_SPEED); // going from no motion to full motion
					this.armTimer.reset();
					this.armTimer.start();
					this.armState = Boolean.TRUE;
					spinningString="Spinning Forward  ";
				}
				else if (axisValue == 1d) {
					this.armSpeedController.set(ARM_REVERSE_SPEED);
					this.armState = Boolean.FALSE;
					spinningString="Spinning Backwards";
				}
				else {
					spinningString="Not Spinning      ";
				}
		}
/*		if (this.isArmSpinningForward) {
			if (this.armTimer.get()>= ARM_TIMEOUT_THRESHOLD) {
				this.armSpeedController.set(ARM_NO_SPEED);
				this.armTimer.stop();
				this.isArmSpinningForward = false;
				this.armState = null;
				spinningString="Not Spinning      ";
			} else {
				this.armState = Boolean.TRUE;
				spinningString="Spinning Forward  ";
			}
		} else {
			double axisValue = this.joystick.getRawAxis(ARM_CONTROL_AXIS);
			
			if (axisValue == -1d) {
				this.armSpeedController.set(ARM_FULL_SPEED); // going from no motion to full motion
				this.isArmSpinningForward = true; //to remember that motor is running
				this.armTimer.reset();
				this.armTimer.start();
				this.armState = Boolean.TRUE;
				spinningString="Spinning Forward  ";
			}
			else if (axisValue == 1d) {
				this.armSpeedController.set(ARM_REVERSE_SPEED);
				this.armState = Boolean.FALSE;
				spinningString="Spinning Backwards";
			}
			else {
				this.armSpeedController.set(ARM_NO_SPEED);
				this.armState = null;
				spinningString="Not Spinning      ";
			}
		} */
		this.driverStationLCD.println(DriverStationLCD.Line.kUser6,1, spinningString);
	}
	
	private void updateDashboard() {
        Dashboard lowDashData = DriverStation.getInstance().getDashboardPackerLow();
        lowDashData.addCluster();
        {
            lowDashData.addCluster();
            {     //analog modules
                lowDashData.addCluster();
                {
                    for (int i = 1; i <= 8; i++) {
                        lowDashData.addFloat((float) AnalogModule.getInstance(1).getAverageVoltage(i));
                    }
                }
                lowDashData.finalizeCluster();
                /*lowDashData.addCluster();
                {
                    for (int i = 1; i <= 8; i++) {
                        lowDashData.addFloat((float) AnalogModule.getInstance(2).getAverageVoltage(i));
                    }
                }
                lowDashData.finalizeCluster();*/
            }
            lowDashData.finalizeCluster();

            lowDashData.addCluster();
            { //digital modules
                lowDashData.addCluster();
                {
                    lowDashData.addCluster();
                    {
                        int module = 1;
                        lowDashData.addByte(DigitalModule.getInstance(module).getRelayForward());
                        lowDashData.addByte(DigitalModule.getInstance(module).getRelayForward());
                        lowDashData.addShort(DigitalModule.getInstance(module).getAllDIO());
                        lowDashData.addShort(DigitalModule.getInstance(module).getDIODirection());
                        lowDashData.addCluster();
                        {
                            for (int i = 1; i <= 10; i++) {
                                lowDashData.addByte((byte) DigitalModule.getInstance(module).getPWM(i));
                            }
                        }
                        lowDashData.finalizeCluster();
                    }
                    lowDashData.finalizeCluster();
                }
                lowDashData.finalizeCluster();

                /*lowDashData.addCluster();
                {
                    lowDashData.addCluster();
                    {
                        int module = 2;
                        lowDashData.addByte(DigitalModule.getInstance(module).getRelayForward());
                        lowDashData.addByte(DigitalModule.getInstance(module).getRelayReverse());
                        lowDashData.addShort(DigitalModule.getInstance(module).getAllDIO());
                        lowDashData.addShort(DigitalModule.getInstance(module).getDIODirection());
                        lowDashData.addCluster();
                        {
                            for (int i = 1; i <= 10; i++) {
                                lowDashData.addByte((byte) DigitalModule.getInstance(module).getPWM(i));
                            }
                        }
                        lowDashData.finalizeCluster();
                    }
                    lowDashData.finalizeCluster();
                }
                lowDashData.finalizeCluster();*/

            }
            lowDashData.finalizeCluster();

            //lowDashData.addByte(Solenoid.getAllFromDefaultModule());
        }
        lowDashData.finalizeCluster();
        lowDashData.commit();

    }

	private void teleopScooper() {
		boolean shouldScoopUp = this.joystick.getRawButton(1);
		boolean shouldScoopDown = this.joystick.getRawButton(4);
		if (shouldScoopUp) {
			if (this.scoopTicks==0){
				this.scoopSpeedController.set(SCOOP_UP_SPEED);
				this.scoopTicks++;
			}
			this.scoopSpeedController.set(SCOOP_UP_SPEED);
			System.out.println("were   scooping up");	
		} 
		else if (shouldScoopDown) {
			this.scoopSpeedController.set(SCOOP_DOWN_SPEED);
			System.out.println("were scooping down");
		}
		else {
			this.scoopSpeedController.set(NOT_SCOOPING);
			this.scoopTicks=0;
		}
	}			
}

