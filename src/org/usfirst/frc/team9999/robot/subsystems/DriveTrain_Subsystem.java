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
		 //Inversion
		 port.setInverted(false);
		 port_slave.setInverted(true);
		 starboard.setInverted(false);
		 starboard_slave.setInverted(true);
		 //Follow
		 starboard_slave.follow(starboard);
		 port_slave.follow(port);
		 //Put stuff on Dash
		 SmartDashboard.putNumber("RPM Port-side", port.getSelectedSensorVelocity(0));
		 SmartDashboard.putNumber("RPM Starboard", starboard.getSelectedSensorVelocity(0));
	 }
	 public void teleopDrive(XboxController control) {
		 mainWCD.arcadeDrive(control.getX(), control.getY(), false);
	 }


    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new arcade_Drive());
    }
    
    public void Stop() {
    
    }
}

