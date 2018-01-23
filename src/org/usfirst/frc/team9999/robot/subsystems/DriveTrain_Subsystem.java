package org.usfirst.frc.team9999.robot.subsystems;

import org.usfirst.frc.team9999.robot.RobotMap;
import org.usfirst.frc.team9999.robot.commands.arcade_Drive;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTrain_Subsystem extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private WPI_TalonSRX starboard = new WPI_TalonSRX(RobotMap.frMotor);
	private WPI_TalonSRX port = new WPI_TalonSRX(RobotMap.flMotor);
	private WPI_VictorSPX starboard_slave = new WPI_VictorSPX(RobotMap.brMotor);
	private WPI_VictorSPX port_slave = new WPI_VictorSPX(RobotMap.blMotor);

	 DifferentialDrive mainWCD = new DifferentialDrive(starboard, port);
	
	 public void robotInit() {
		 //Sens=a=tech
		 //feedbackDevice - Remote Feedback Device to select.
		 //pidIdx - 0 for Primary closed-loop. 1 for cascaded closed-loop.
		 /*timeoutMs - Timeout value in ms. If nonzero, function will wait for config success and report an error if it times out. 
		 If zero, no blocking or checking is performed.*/
		 port.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		 starboard.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		 /* set the peak and nominal outputs, 12V means full */
		 port.configNominalOutputForward(1, 10); //probably want to try 12v here, but this is copied from the manual
		 port.configNominalOutputReverse(-1, 10);
		 port.configPeakOutputForward(1, 10);
		 starboard.configNominalOutputForward(0, 10);
		 starboard.configNominalOutputReverse(0, 10);
		 starboard.configPeakOutputForward(1, 10);
		 //Inversion
		 port.setInverted(false);
		 port_slave.setInverted(true);
		 starboard.setInverted(false);
		 starboard_slave.setInverted(true);
		 //Config PID
		 port.selectProfileSlot(0, 10);
		 port.config_kF(0, 0, 10); //Feed-Foward Gain (This is only found in motion profiling apps
		 port.config_kP(0, 0, 10); //These are the regular PID gains
		 port.config_kI(0, 0, 10);
		 port.config_kD(0, 0, 10);
		 //Follow
		 starboard_slave.follow(starboard);
		 port_slave.follow(port);
		 //Put stuff on Dash
		 SmartDashboard.putNumber("RPM Port-side", port.getSelectedSensorVelocity(0));
		 SmartDashboard.putNumber("RPM Starboard", starboard.getSelectedSensorVelocity(0));
	 }
	 public void teleopDrive(XboxController control) {
		 mainWCD.arcadeDrive(control.getRawAxis(1), control.getRawAxis(5), false);
		 //we need to set a mode to follow motion magic here ex. port.setMode(ControlMode.MotionMagic, something);
		 /*"Bang-Bang Control"
		 if(port.getActiveTrajectoryVelocity() > starboard.getActiveTrajectoryVelocity()) {
			 starboard.set(starboard.getActiveTrajectoryVelocity() + (port.getActiveTrajectoryVelocity() - starboard.getActiveTrajectoryVelocity()));
		 }
		 else if (starboard.getActiveTrajectoryVelocity() > port.getActiveTrajectoryVelocity()) {
			 port.set(port.getActiveTrajectoryVelocity() + (starboard.getActiveTrajectoryVelocity() - port.getActiveTrajectoryVelocity()));
		 }
		 */
		 //Find a way to use pid & motion magic to make the drivetrain straight IN TELEOP, people on chiefdelphi brag about how easy it is and refuse to post code...
		
	 }


    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new arcade_Drive());
    }
    
    public void Stop() {
    
    }
}

