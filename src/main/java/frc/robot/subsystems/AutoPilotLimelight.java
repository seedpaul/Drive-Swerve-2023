// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.Arrays;


import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;


public class AutoPilotLimelight extends SubsystemBase {

 
  private static AutoPilotLimelight m_inst = null;

  private final Joystick autoPilotController = new Joystick(Constants.AutoPilotController.AUTOPILOT_CONTROLLER_USB_ID);
  private final JoystickButton button1 = new JoystickButton(autoPilotController, Constants.AutoPilotController.GREEN_LEFT);
  private final JoystickButton button2 = new JoystickButton(autoPilotController, Constants.AutoPilotController.GREEN_CENTER);
  private final JoystickButton button3 = new JoystickButton(autoPilotController, Constants.AutoPilotController.GREEN_RIGHT);
  private final JoystickButton button4 = new JoystickButton(autoPilotController, Constants.AutoPilotController.YELLOW_LEFT);
  private final JoystickButton button5 = new JoystickButton(autoPilotController, Constants.AutoPilotController.YELLOW_CENTER);
  private final JoystickButton button6 = new JoystickButton(autoPilotController, Constants.AutoPilotController.YELLOW_RIGHT);
  private final JoystickButton button7 = new JoystickButton(autoPilotController, Constants.AutoPilotController.BLUE_LEFT);
  private final JoystickButton button8 = new JoystickButton(autoPilotController, Constants.AutoPilotController.BLUE_CENTER);
  private final JoystickButton button9 = new JoystickButton(autoPilotController, Constants.AutoPilotController.BLUE_RIGHT);
  private final JoystickButton buttonBottom = new JoystickButton(autoPilotController, Constants.AutoPilotController.CLEAR_BOTTOM);
  private final JoystickButton buttonMiddle = new JoystickButton(autoPilotController, Constants.AutoPilotController.CLEAR_MIDDLE);
  private final JoystickButton buttonTop = new JoystickButton(autoPilotController, Constants.AutoPilotController.CLEAR_TOP);
  private final JoystickButton[] autoPilotButtons = {button1,button2,button3,button4,button5,button6,button7,button8,button9,buttonBottom,buttonMiddle,buttonTop};

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("FMSInfo");
  private BooleanTopic isRedAlliance = table.getBooleanTopic("IsRedAlliance");
  private BooleanSubscriber IsRed = isRedAlliance.subscribe(false,PubSubOption.keepDuplicates(false),PubSubOption.pollStorage(6));
  private boolean isRed;

  /** Creates a new autopilot. */
  private AutoPilotLimelight() {
     this.isRed = IsRed.get();
  }

  public static AutoPilotLimelight getInstance() {
    if (m_inst == null) {
      m_inst = new AutoPilotLimelight();
    }
    return m_inst;
  }

  @Override
  public void periodic() {

    boolean hasTargets = LimelightHelpers.getTV(Constants.LIMELIGHT_NAME);
    // Check if the latest result has any targets
  
    if(hasTargets){
      double targetId = LimelightHelpers.getFiducialID(Constants.LIMELIGHT_NAME);
      // Get the current best target.
      LimelightHelpers.getBotPose2d(Constants.LIMELIGHT_NAME);
      LimelightHelpers.getBotPose3d(Constants.LIMELIGHT_NAME);

      // Get information from target.
      
      LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(Constants.LIMELIGHT_NAME);
      double confidence = results.targetingResults.targets_Classifier[0].confidence;
      
      if(confidence < 0.2){
        Pose2d currentPosition = LimelightHelpers.getBotPose2d(Constants.LIMELIGHT_NAME);
        
        if(this.locationSpecified() && this.heightSpecifed()){      
          Pose2d getTargetPosition = getScoringLocation();
  
          Trajectory autoPath = TrajectoryGenerator.generateTrajectory(currentPosition, null, getTargetPosition, new TrajectoryConfig(3.0, 4.0));
        }
      }
      SmartDashboard.putString("getBotPose2d",LimelightHelpers.getBotPose2d(Constants.LIMELIGHT_NAME).toString());
    }
  }

  // private Pose2d calculateRobotPose(int targetID, Transform3d bestCameraToTarget){

  //   Pose2d tagPose = Constants.AprilTagCoordinates.getAprilTagPose(targetID);
  //   return tagPose.plus(new Transform2d(bestCameraToTarget.getTranslation().toTranslation2d(),bestCameraToTarget.getRotation().toRotation2d()));
  // }

  private Pose2d getScoringLocation(){
    return Constants.ScoringLocations.getLocation(this.whatLocation(Arrays.copyOfRange(autoPilotButtons, 0, 8)), isRed).pose;
  }

  private boolean locationSpecified(){

     return (button1.getAsBoolean() || button2.getAsBoolean() || button3.getAsBoolean() || button4.getAsBoolean() || 
      button5.getAsBoolean() || button6.getAsBoolean() || button7.getAsBoolean() || button8.getAsBoolean() || button9.getAsBoolean())&& singleButtonfromGroup(Arrays.copyOfRange(autoPilotButtons, 0, 8));
  }

  private boolean heightSpecifed(){
    return (buttonTop.getAsBoolean() || buttonMiddle.getAsBoolean() || buttonBottom.getAsBoolean()) && singleButtonfromGroup(Arrays.copyOfRange(autoPilotButtons, 9, 11));
  }

  private boolean singleButtonfromGroup(JoystickButton[] buttons){
    int count = 0;
    for(int i=0; i< buttons.length; i++){
      if(buttons[i].getAsBoolean()){
        count++;
      }
    }

    return count == 1;
  }

  private int whatLocation(JoystickButton[] buttons){
    int id = 0;
    for(int i=0; i< buttons.length; i++){
      if(buttons[i].getAsBoolean()){
        id = i+1;
      }
    }

    return id;
  }

  private int whatHeight(JoystickButton[] buttons){
    int id = 0;
    for(int i=0; i< buttons.length; i++){
      if(buttons[i].getAsBoolean()){
        id = i+10;
      }
    }

    return id;
  }
}