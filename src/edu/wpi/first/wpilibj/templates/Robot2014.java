/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
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
    private static final double ARM_REVERSE_SPEED = -0.3d;
	private static final double ARM_NO_SPEED = 0d;
	private static final double ARM_TIMEOUT_THRESHOLD = 0.65d;
	/* member variables */
    private Joystick joystick;
    private DriverStationLCD driverStationLCD;
    private SpeedController armSpeedController;
	private boolean isArmSpinningForward;
	private Timer armTimer;
	private RobotDrive robotDrive;
    private SpeedController leftDriveSpeedController;
	private SpeedController rightDriveSpeedController;
	
	 
	/**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        this.joystick = new Joystick(JOYSTICK_USB_PORT);
        this.driverStationLCD = DriverStationLCD.getInstance();
		this.isArmSpinningForward = false;
		this.armTimer = new Timer();
		this.armSpeedController = new Victor(3);
		this.leftDriveSpeedController = new Talon(1);
		this.rightDriveSpeedController = new Talon(6);
		this.robotDrive = new RobotDrive(this.leftDriveSpeedController, this.rightDriveSpeedController);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
// the button -when axis 3=-1, motor will run only for 1 second
		if (this.isArmSpinningForward) {
			if (this.armTimer.get()>= ARM_TIMEOUT_THRESHOLD) {
				this.armSpeedController.set(ARM_NO_SPEED);
				this.armTimer.stop();
				this.isArmSpinningForward = false;
			}
		} else {
			double axisValue = this.joystick.getRawAxis(ARM_CONTROL_AXIS);

			if (axisValue == -1d) {
				this.armSpeedController.set(ARM_FULL_SPEED); // going from no motion tofull motion
				this.isArmSpinningForward = true; //to remember that motor is running
				this.armTimer.reset();
				this.armTimer.start();
			} else if (axisValue == 1d) {
				this.armSpeedController.set(ARM_REVERSE_SPEED); // going from no motion tofull motion
			} else {
				this.armSpeedController.set(ARM_NO_SPEED);
			}
		}
		this.driverStationLCD.println(DriverStationLCD.Line.kUser6,1,"" + this.isArmSpinningForward);
		this.driverStationLCD.updateLCD();
		this.robotDrive.arcadeDrive(this.joystick);

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
}
/*
1:012.34567

*/
