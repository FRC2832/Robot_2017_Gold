package org.usfirst.frc2832.Robot2017.autonCommands;

import org.usfirst.frc2832.Robot2017.DriveEncoders;
import org.usfirst.frc2832.Robot2017.Robot;
import org.usfirst.frc2832.Robot2017.subsystems.DriveTrain;
import org.usfirst.frc2832.Robot2017.subsystems.NavX;

import edu.wpi.first.wpilibj.command.Command;

/**
 * MUST BE TESTED ON RIGHT, MAY NOT WORK ON THE RIGHT
 * New routine
 */
public class RotateNavXWithEnc extends Command {

	private double rot, initDeg;
	private int dispCount=0;
	private double encChange = 1170; //VALUE FOR THE PRACTICE ROBOT! MUST TEST FOR ACTUAL BOT
	private double initEnc;

	
    public RotateNavXWithEnc(double rot) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis)
    	this.rot = rot;
    	requires(Robot.driveTrain);
    }
    
    public RotateNavXWithEnc(double rot, double changeEnc) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis)
    	this.rot = rot;
    	requires(Robot.driveTrain);
    	encChange = changeEnc;
    }
    // Called just before this Command runs the first time
    protected void initialize() {
    	//System.out.println("RotateNavX Start");
    	initDeg = NavX.getHeading();
    	dispCount=0;
    	if (rot > 0)
    		initEnc = DriveEncoders.getRawLeftValue();
    	else
    		initEnc = DriveEncoders.getRawRightValue();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(rot > 0) {
    		DriveTrain.robotDrive.arcadeDrive(0.1, 0.6);
    		//Robot.driveTrain.setTankDriveCommand(0.5, -0.5);
    		//System.out.println(">");
    		//Robot.driveTrain.setTankDriveCommand(-0.2 * (rot - initDeg), 0.2 * (initDeg - rot));
    	} else {
    		DriveTrain.robotDrive.arcadeDrive(0.1, -0.6);
    		//Robot.driveTrain.setTankDriveCommand(-0.5, 0.5);
    		//System.out.println("<");
    		//Robot.driveTrain.setTankDriveCommand(-0.2 * (initDeg - rot), 0.2 * (initDeg - rot));
    	}
    	if (dispCount==10) { 
    		System.out.println(NavX.getHeading() + ":	" + initDeg + ":	" + rot);
    		if (rot > 0)
    			System.out.println("EncoderTurn:" + initEnc + ", " + DriveEncoders.getRawLeftValue() + ", " + encChange);
    		else 
    			System.out.println("EncoderTurn:" + initEnc + ", " + DriveEncoders.getRawRightValue() + ", " + encChange);
    		dispCount=0;
    		
    	}
    	else dispCount++;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	
    	
    	if (rot > 0) {
    		if (NavX.getHeading() > normalize(initDeg + rot)){
    			System.out.println("Stop by NavX");
				return (NavX.getHeading() > normalize(initDeg + rot));
    		}
			else if ((DriveEncoders.getRawLeftValue() - initEnc > encChange)){ 
				System.out.println("Stop by Encoders");
				return(DriveEncoders.getRawLeftValue() - initEnc > encChange);
			}
    	} else {
    		if  (NavX.getHeading() < normalize(initDeg + rot)){
    			System.out.println("Stop by NavX");
    			return NavX.getHeading() < normalize(initDeg + rot);
    		}
    		else if (initEnc - DriveEncoders.getRawRightValue() > encChange)
    		{
    			System.out.println("Stop by Encoders");
    			return initEnc - DriveEncoders.getRawRightValue() > encChange;
    			
    		}
    	}
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	DriveEncoders.intializeEncoders();
    	//System.out.println("RotateNavX End");

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {

    }
    
    private double normalize(double input) {
		double normalizedValue = input;
		while (normalizedValue > 180)
			normalizedValue -= 360;
		while (normalizedValue < -180)
			normalizedValue += 360;

		return normalizedValue;
	}
}
