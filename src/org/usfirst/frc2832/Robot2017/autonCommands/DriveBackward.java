package org.usfirst.frc2832.Robot2017.autonCommands;

import org.usfirst.frc2832.Robot2017.Robot;
import org.usfirst.frc2832.Robot2017.DriveEncoders;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * added a timeout 4/7/17
 */
public class DriveBackward extends Command {
	private double distance= 2;//to cross green line distnace in meters is approximately 2.41097 so set it to 2.5 meters
	private double initEncoderVal =0;
	private double startTime;//added at CHS competition
	private double endTime;
	

     public DriveBackward() {        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    	}
    
    
     public DriveBackward(double distance) {
        // Use requires() here to declare subsystem dependencies
        
    	this();
    	setDistance(distance);
    	
    	}
     
     

    public double getDistance() {
		return distance;
	}


	public void setDistance(double straightDistance) {
		this.distance = straightDistance;
	}


	public double getInitEncoderVal() {
		return initEncoderVal;
	}


	public void setInitEncoderVal(double initEncoderVal) {
		this.initEncoderVal = initEncoderVal;
	}
	


	// Called just before this Command runs the first time
    protected void initialize() {
    	setInitEncoderVal(DriveEncoders.getAbsoluteValue());
    	startTime = Timer.getFPGATimestamp();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveTrain.setArcadeDriveCommand(-0.5, 0.0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return (Math.signum(getDistance()) * (getInitEncoderVal() - DriveEncoders.getAbsoluteValue()) > Math.abs(getDistance())) || ((Timer.getFPGATimestamp() - startTime) > 2) ;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.setArcadeDriveCommand(0.0, 0.0);

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
