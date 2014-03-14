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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Watchdog;


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
	private static final int LEFT_A_CHANNEL_D_I_O_PORT = 3;
	private static final int LEFT_B_CHANNEL_D_I_O_PORT = 4;
	private static final int RIGHT_A_CHANNEL_D_I_O_PORT = 6;
	private static final int RIGHT_B_CHANNEL_D_I_O_PORT = 7;
	private static final int ROBOT_MOVEMENT_AXIS = 2;
	private static final int ROBOT_ROTATE_AXIS = 1;
	private static final boolean SQUARED_INPUTS = true;
	private static final boolean LEFT_ENCODER_REVERSE = true;
	private static final boolean RIGHT_ENCODER_REVERSE = false;
	private static final double SCOOP_UP_SPEED = .65d;
	private static final double SCOOP_DOWN_SPEED = -0.25d;
	private static final double NOT_SCOOPING = 0d;
	private static final int TICK_START = 10;
	private static final int ROTATION_INVERSION_D_S_D_I = 1;
	private static final int AUTONOMOUS_MODE_D_S_D_I = 8;
	private static final int AUTO_HAMMER_D_S_D_I = 7;
	/* member variables */
    private GenericHID joystick;
    private DriverStationLCD driverStationLCD;
    private DriverStation driverStation;
	private Watchdog watchdog;
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
	private int autonomousState;
	private Timer autonomousTimer;
	private boolean autonomousMode;
	private boolean autohammer;
	private int autoArmState;
	private Timer autoArmTimer;
	private Encoder leftEncoder;
	private Encoder rightEncoder;
	/**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        this.joystick = new Joystick(JOYSTICK_USB_PORT);
        this.driverStationLCD = DriverStationLCD.getInstance();
		this.driverStation = DriverStation.getInstance();
		this.watchdog = Watchdog.getInstance();
		this.armSpeedController = new Victor(3);
		this.scoopSpeedController = new Talon(4);
		this.armTimer = new Timer();
		this.photoSensorDigitalInput = new DigitalInput(PHOTO_SENSOR_D_I_O_PORT);
		this.leftDriveSpeedController = new Talon(LEFT_DRIVE_SPEEDCONTROLLER_PWM_PORT);
		this.rightDriveSpeedController = new Talon(RIGHT_DRIVE_SPEEDCONTROLLER_PWM_PORT);
		this.robotDrive = new RobotDrive(this.leftDriveSpeedController, this.rightDriveSpeedController);
		this.autoArmTimer = new Timer();
		this.leftEncoder = new Encoder(LEFT_A_CHANNEL_D_I_O_PORT, LEFT_B_CHANNEL_D_I_O_PORT, LEFT_ENCODER_REVERSE);
		this.rightEncoder = new Encoder(RIGHT_A_CHANNEL_D_I_O_PORT, RIGHT_B_CHANNEL_D_I_O_PORT, RIGHT_ENCODER_REVERSE);

	}

	public void autonomousInit() {
		this.autonomousTimer = new Timer();
		this.autonomousState = 0;
		this.autonomousTimer.reset();
		this.autonomousTimer.start();
		this.autonomousMode = this.driverStation.getDigitalIn(AUTONOMOUS_MODE_D_S_D_I);
		this.autohammer = this.driverStation.getDigitalIn(AUTO_HAMMER_D_S_D_I);
		this.driverStationLCD.println(DriverStationLCD.Line.kUser2, 1, "auto-hammer="+this.autohammer);
		this.leftEncoder.reset();
		this.rightEncoder.reset();
		this.leftEncoder.start();
		this.rightEncoder.start();
	}
	/**
     * This function is called periodically during autonomous
     */
   public void autonomousPeriodic() {
		this.watchdog.feed();
		int leftEncoderCount = this.leftEncoder.get();
		int rightEncoderCount = this.rightEncoder.get();
		if(this.autonomousMode){
			this.autonomousModeBasic();
		}
		else {
			this.autonomousModeAdvanced(leftEncoderCount, rightEncoderCount);
		}
		
		this.driverStationLCD.println(DriverStationLCD.Line.kUser3, 1, "LE count: "+leftEncoderCount);
		this.driverStationLCD.println(DriverStationLCD.Line.kUser4, 1, "RE count: "+rightEncoderCount);
		this.driverStationLCD.updateLCD();
	}

	private void autonomousModeAdvanced(int leftEncoderCount, int rightEncoderCount) {
		double moveValue;
		double rotateValue;
		int leftCountRem= -5400-leftEncoderCount;
		int rightCountRem= -5400-rightEncoderCount;
		this.driverStationLCD.println(DriverStationLCD.Line.kUser1, 1, "autonomousState=" + this.autonomousState);
		switch (this.autonomousState) {
			case 0:
				//this.scoopSpeedController.set(SCOOP_DOWN_SPEED);
				this.autonomousState++;
				this.driverStationLCD.updateLCD();
				break;
			case 1:
				moveValue = .6d;
				rotateValue = this.calculateRotation(leftCountRem, rightCountRem);
				this.robotDrive.arcadeDrive( moveValue, rotateValue); 
				
				if (this.autonomousTimer.get() >= 1d ){
					this.scoopSpeedController.set(0);
				}
					
				if (this.autonomousTimer.get() >= 6d ){
					moveValue = 0d;
					rotateValue = 0d;
					this.robotDrive.arcadeDrive( moveValue, rotateValue);
					if (this.autohammer){
						this.armSpeedController.set(ARM_REVERSE_SPEED);
					}
					this.autonomousState++;
					this.driverStationLCD.updateLCD();
				}
				break;
			case 2:
				if (this.autonomousTimer.get() >= 6.5){
					if (this.autohammer){
						this.armSpeedController.set(ARM_FULL_SPEED);
					}
					this.autonomousState++;
					this.driverStationLCD.updateLCD();
				}
				break;
			case 3:
				if (this.autonomousTimer.get() >= 7){
					this.armSpeedController.set(ARM_NO_SPEED);
					this.autonomousState++;
					this.driverStationLCD.updateLCD();
				}
				break;
			default:
				this.autonomousTimer.stop();
				this.driverStationLCD.updateLCD();
				break;
		}
	}
   
    public double calculateRotation(int leftEncoderCount, int rightEncoderCount){
		if ((leftEncoderCount==0)) {
			return -1d;
		}
		double reDouble = rightEncoderCount;
		double leDouble = leftEncoderCount;
		double encoderRatio = 1d-(reDouble/leDouble);
		this.driverStationLCD.println(DriverStationLCD.Line.kUser6, 1,"rotateValue= "+encoderRatio);
		this.driverStationLCD.updateLCD();
		return encoderRatio;
		
	} 
	
	
	public void autonomousModeBasic(){
		double moveValue;
		double rotateValue;
		switch (this.autonomousState) {
			case 0:
				moveValue = -.6d;
				rotateValue = -.2d;
				this.robotDrive.arcadeDrive( moveValue, rotateValue);
				if (this.autonomousTimer.get() >= 3d ){
					moveValue = 0d;
					rotateValue = 0d;
					this.robotDrive.arcadeDrive( moveValue, rotateValue);
					this.autonomousTimer.stop();
					this.autonomousState++;
				}
				break;
			
			default:
				break;
		}
   }
	
	/** 
	 * This function is called once you hit enable for periodic mode
	 */
	public void teleopInit() {
		this.driverStationLCD.clear();
		this.invertDriveRotation = this.driverStation.getDigitalIn(ROTATION_INVERSION_D_S_D_I);
		this.driverStationLCD.println(DriverStationLCD.Line.kUser1, 1, (this.invertDriveRotation)?"Inverted Rotation    ":"Not inverted rotation");
		this.driverStationLCD.updateLCD();
		this.autoArmState = 3;
		this.leftEncoder.reset();
		this.rightEncoder.reset();
		this.leftEncoder.start();
		this.rightEncoder.start();
	}
    
	public void teleopDrive(){
		double moveValue = this.joystick.getRawAxis(ROBOT_MOVEMENT_AXIS);
		double rotateValue = this.joystick.getRawAxis(ROBOT_ROTATE_AXIS);
		this.robotDrive.arcadeDrive( moveValue, ((this.invertDriveRotation)?-1d:1d)*rotateValue, SQUARED_INPUTS);
		int leftEncoderCount = this.leftEncoder.get();
		int rightEncoderCount = this.rightEncoder.get();
		this.driverStationLCD.println(DriverStationLCD.Line.kUser3, 1, "LE count: "+leftEncoderCount);
		this.driverStationLCD.println(DriverStationLCD.Line.kUser4, 1, "RE count: "+rightEncoderCount);
		
	} 
	
	
	
	/**
     * This function is called periodically during operator control
     */
	public void teleopPeriodic() {
		this.watchdog.feed();
		this.teleopDrive();
		this.teleopScooper();
		this.teleopArm();
		this.teleopAutoArm();
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
		this.watchdog.feed();
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
		}
		this.driverStationLCD.updateLCD();    
	}
	
	public void teleopAutoArm(){
		if(this.armState!=null){
			return;
		}
		switch(this.autoArmState){
			case 0:
				this.autoArmTimer.reset();
				this.autoArmTimer.start();
				this.armSpeedController.set(ARM_REVERSE_SPEED);
				this.autoArmState++;		
				break;
			case 1:
				if (this.autoArmTimer.get() >= .5d){
					this.armSpeedController.set(ARM_FULL_SPEED);
					this.autoArmState++;
				}
				break;
			case 2:
				if (this.autoArmTimer.get() >= 1d){
					this.armSpeedController.set(ARM_NO_SPEED);
					this.autoArmState++;
					this.autoArmTimer.stop();
				}
			default:
				if (this.joystick.getRawAxis(ARM_CONTROL_AXIS)==1d){
					this.autoArmState = 0;				
				}
				break;
			}
	}
	
	private void teleopArm() {
	// the button -when axis 3=-1, motor will run only for 1 second
		String spinningString;
		if (this.autoArmState <3){
			return;
		}
		switch ((this.armState == null)?-1:(this.armState.booleanValue())?1:0) {
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
				if (!this.photoSensorDigitalInput.get()) {
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
				//else if (axisValue == 1d) {
					//this.armSpeedController.set(ARM_REVERSE_SPEED);
					//this.armState = Boolean.FALSE;
					//spinningString="Spinning Backwards";
				//}
				else {
					spinningString="Not Spinning      ";
				}
		}
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
			//if (this.scoopTicks==0){
				this.scoopSpeedController.set(SCOOP_UP_SPEED);
				//this.scoopTicks++;
			//}
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

