package org.usfirst.frc.team20.robot;

import java.io.IOException;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot  implements PIDOutput {
	//drive pid values for turning
	    
		RobotDrive MyDrive;
	    PIDController turnController;   
	    double rotateToAngleRate;
	    SpeedControllers speedController;
	    int group;
	    int state;
	    int auto;
	    AHRS hrs;
	    Utils util;
	    
	    float angleFromCamera = 0.00f;
	    double distanceFromCamera = 0.00;
	  
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		auto = AutoModes.AUTO_MODE_1;
		hrs = new AHRS(SerialPort.Port.kMXP);
		util = new Utils();
		speedController = new SpeedControllers();
		speedController.setDriveTalons();
		MyDrive = new RobotDrive(speedController.rightMaster,speedController.leftMaster);
		turnController = new PIDController(PidTurnValues.kP, PidTurnValues.kI, PidTurnValues.kD, PidTurnValues.kF, hrs,this);
		turnController.setInputRange(-180.0f,  180.0f);
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setAbsoluteTolerance(PidTurnValues.kToleranceDegrees);
		turnController.setContinuous(true);
		
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		// auto mode call
		auto = AutoModes.AUTO_MODE_1;
		// **********************
         group = Groups.GROUP_1;
	     hrs.reset();
	 	 state = States.nothing;
	 	 try {
			util.createFile("test.txt");
			util.WriteToFile("testeeeee44455");
			util.Closefile();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		   switch (auto) {
       		case AutoModes.AUTO_MODE_1 :
       			run_Auto01();
       		//	System.out.println("hrs angle =" + hrs.getAngle());
       		  
       		break;
       	default :
       		System.out.println("AUTO MODE = " + auto);
       }

	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		hrs.reset();
	//	hrs = new AHRS(SerialPort.Port.kMXP);
	//	util = new Utils();
	//	speedController = new SpeedControllers();
	//	speedController.setDriveTalons();
	//	MyDrive = new RobotDrive(speedController.rightMaster,speedController.leftMaster);
		
		
	}
	
// user methods begin
/*	private void setTurnController(double turnAngle) {
	    turnController.setInputRange(-180.0f,  180.0f);
	    turnController.setOutputRange(-1.0, 1.0);
	    turnController.setAbsoluteTolerance(PidTurnValues.kToleranceDegrees);
	    turnController.setContinuous(true);
	    turnController.setSetpoint(turnAngle);
	    turnController.enable();
	   
	} */
	/*
	private boolean TurnAngle2(float cameraAngle )
	{	
		boolean doneTurning = false;
		if (Math.abs(cameraAngle - hrs.getAngle()) < .5)
		{
			MyDrive.arcadeDrive(0.0, 0);
			doneTurning = true;
		}
		else 
		{
			if (cameraAngle < 0 )
				MyDrive.arcadeDrive(0.0, -.5);
			else 
				MyDrive.arcadeDrive(0.0, .4);
		}
			
		System.out.println("Camera Angle = " + cameraAngle );
		System.out.println("NavX Angle = " + hrs.getAngle() );
		return doneTurning;
	}
	*/
	
	private boolean TurnAngle(float cameraAngle )
	{
		System.out.println("test camera**************************");
		double angle = cameraAngle;
		boolean doneTurning = false;
		double	currentRotationRate = rotateToAngleRate;
		 if (Math.abs(angle - hrs.getAngle()) < .6 && Math.abs(currentRotationRate) < .3){
				currentRotationRate = 0;
				MyDrive.arcadeDrive(0.0, 0);
				//turnController.disable();
				doneTurning = true;	
			}
		 else
		 {
			 try {
				 MyDrive.arcadeDrive(0.0, currentRotationRate);
				 //Timer.delay(0.004);
				 System.out.println("Navx " + hrs.getAngle());
				 System.out.println(" currentRotationRate  = " + currentRotationRate);
				 System.out.println(" Math.abs(angle - hrs.getAngle()) = " +  Math.abs(angle - hrs.getAngle()) );
			 	} catch ( RuntimeException ex ) {
			 		DriverStation.reportError("Error communicating with drive system: " + ex.getMessage(), true);
			 	}
		 }
			 //if (Math.abs(currentRotationRate) < .28 && Math.abs(angle - hrs.getAngle()) < .5 ){
		
			 return doneTurning;
	}
	/*
	public boolean driveStraight(double speed, double inches,double multiplier){
		boolean doneDriving = false;
		System.out.println("Speed " + speed); // .5
		System.out.println("multiplier " + multiplier); // 6.5
		System.out.println("distance" + inches); // 80
		double	currentRotationRate = rotateToAngleRate;
		System.out.println("CurrentRR: " + currentRotationRate);
		if(speedController.leftMaster.getEncPosition()/1024*Math.PI*4 > (inches*multiplier)){
			System.out.println("Done");
			 MyDrive.arcadeDrive(0, 0);
			//speedController.leftMaster.set(0);
			//speedController.rightMaster.set(0);
			doneDriving = true;
			//leftMaster.set(0);
			//rightMaster.set(0);
		}
		else{
			  MyDrive.arcadeDrive(.6, currentRotationRate);
			  Timer.delay(0.004);
			//speedController.rightMaster.set(.66);
			//speedController.leftMaster.set(-.6);
			 System.out.println("Navx = " + hrs.getAngle());
		}
		
		return doneDriving;
	}*/

	public boolean driveStraight(double speed, double inches,double multiplier){
		boolean doneDriving = false;
		System.out.println("Speed " + speed); // .5
		System.out.println("multiplier " + multiplier); // 6.5
		System.out.println("distance" + inches); // 80
		double	currentRotationRate = rotateToAngleRate;
		System.out.println("CurrentRR: " + currentRotationRate);
		if(speedController.leftMaster.getEncPosition()/1024*Math.PI*4 > (inches*multiplier)){
			System.out.println("Done");
			 MyDrive.arcadeDrive(0, 0);
			//speedController.leftMaster.set(0);
			//speedController.rightMaster.set(0);
			doneDriving = true;
			//leftMaster.set(0);
			//rightMaster.set(0);
		}
		else{
			  MyDrive.arcadeDrive(.65, currentRotationRate);
			 // Timer.delay(0.004);
			//speedController.rightMaster.set(.66);
			//speedController.leftMaster.set(-.6);
			 System.out.println("Navx = " + hrs.getAngle());
		}
		
		return doneDriving;
	}	
	
	
	@Override
	public void pidWrite(double output) {
		 rotateToAngleRate = output;
	}	
	
// user methods end
	
	// Auto Modes

	// turn an angle
	// drive straight
	
public void run_Auto01()
{
	if (state == States.nothing && group == Groups.GROUP_1){
		String getSocketData;
		hrs.reset();
		//distanceFromCamera
		getSocketData = util.getCameraAngle();
		String [] values = getSocketData.split("\\*");
		distanceFromCamera = Double.parseDouble(values[0]);
		angleFromCamera =  Float.parseFloat(values[1]);
	    turnController.setSetpoint(angleFromCamera);
	    turnController.enable();
	    state = States.TURN_ANGLE;
	    group = Groups.GROUP_1;
	    
	}
	if (state == States.TURN_ANGLE && group == Groups.GROUP_1){
		System.out.println("**********************************angleFromCamera = "   + angleFromCamera);
		System.out.println("**********************************Navx = "   + hrs.getAngle());

		if(TurnAngle(angleFromCamera )){
			state = States.GO_DISTANCE;
			group = Groups.GROUP_2;
			
			//System.out.println("Finish**********************************angleFromCamera = "   + angleFromCamera);
			//System.out.println("Finish**********************************Navx = "   + hrs.getAngle());
			//set encoder to zero so it can calculate distance
			speedController.leftMaster.setEncPosition(0);
		}
	}
	
	if (state == States.DONE && group == Groups.GROUP_2) {
	/*	MyDrive.arcadeDrive(0,0);
		System.out.println("Set distance parameters");
		 hrs.reset();
		 turnController.disable();
		 setTurnController(0.00f);
		 Timer.delay(.02);
		 state = States.GO_DISTANCE;
		//if(driveStraight(.5, 20.00,6.5))
		 System.out.println("Done Set distance parameters");	
		//	state = States.DONE;  */
		MyDrive.arcadeDrive(0,0);
		hrs.reset();
		System.out.println("UUUUUUUUUU " + hrs.getAngle());
	}
	
	if (state == States.GO_DISTANCE && group == Groups.GROUP_2){
		System.out.println("*******lkjljlkjlkj;jl;dj;kafjf;ad************Go distance parameters");
		if (driveStraight(.8, distanceFromCamera,6.5))
		{
			
			state = States.DONE;
			System.out.println("NAVx Angle at end: " + hrs.getAngle());
		}
		
		
	}
	
}
/*
public void run_Auto02()
{	
	if(state == States.nothing && group == Groups.GROUP_1) {
		setTurnController(0.0f);
		speedController.leftMaster.setEncPosition(0);
		state = States.GO_DISTANCE;
	}
	if(state == States.GO_DISTANCE && group == Groups.GROUP_1) {
		double firstDistance = 80.0;
		if (driveStraight(.7, firstDistance,6.5))
		{
			
			state = States.nothing;
			group = Groups.GROUP_2;
			
		}
	}
	if(state == States.nothing && group == Groups.GROUP_2) {
		hrs.reset();
		setTurnController(90.0f);
		state = States.TURN_ANGLE;
	}
	if(state == States.TURN_ANGLE && group == Groups.GROUP_2) {
		if(TurnAngle(90.0f)) {
			state = States.nothing;
			group = Groups.GROUP_3;
		}
	}
	if (state == States.nothing && group == Groups.GROUP_3){
		String getSocketData;
		//distanceFromCamera
		//angleFromCamera = util.getCameraAngle();
		getSocketData = util.getCameraAngle();
		String [] values = getSocketData.split("\\*");
		distanceFromCamera = Double.parseDouble(values[0]);
		angleFromCamera =  Float.parseFloat(values[1]);
		hrs.reset();
	    setTurnController(angleFromCamera);
	    
	    state = States.TURN_ANGLE;
	}
	if (state == States.TURN_ANGLE && group == Groups.GROUP_3){
		System.out.println("**********************************angleFromCamera = "   + angleFromCamera);
		System.out.println("**********************************Navx = "   + hrs.getAngle());

		if(TurnAngle(angleFromCamera )){
			state = States.GO_DISTANCE;
			System.out.println("Finish**********************************angleFromCamera = "   + angleFromCamera);
			System.out.println("Finish**********************************Navx = "   + hrs.getAngle());
			//set encoder to zero so it can calculate distance
			speedController.leftMaster.setEncPosition(0);
			group = Groups.GROUP_4;
			state = States.nothing;
		}
	}
	if (state == States.nothing && group == Groups.GROUP_4) {
		hrs.reset();
		 setTurnController(0.00f);
		 state = States.GO_DISTANCE;
		//if(driveStraight(.5, 20.00,6.5))
			
		//	state = States.DONE;
	}
	
	if (state == States.GO_DISTANCE && group == Groups.GROUP_4){
		if (driveStraight(.7, distanceFromCamera,6.5))
		{
			
			state = States.DONE;
			turnController.disable();
			
		}
		
		
	}
	
}*/
}

