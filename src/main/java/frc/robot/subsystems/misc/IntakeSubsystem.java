package frc.robot.subsystems.misc;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants.IntakeSubsystemConstants;
import frc.robot.subsystems.utility.Sensors;

public class IntakeSubsystem extends SubsystemBase {
    
    //Intake Motor
        private final TalonFX intakeMotor = new TalonFX(IntakeSubsystemConstants.INTAKE_MOTOR_ID);
        private final SparkMax intakeSliderMotor = new SparkMax(IntakeSubsystemConstants.INTAKE_PIVOT_MOTOR_ID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
            private final SparkAbsoluteEncoder intakeSliderEncoder = intakeSliderMotor.getAbsoluteEncoder();
    
    //PID - Intake Slider
        private final PIDController sliderPID = new PIDController(IntakeSubsystemConstants.INTAKE_SLIDER_kP, IntakeSubsystemConstants.INTAKE_SLIDER_kI, IntakeSubsystemConstants.INTAKE_SLIDER_kD);    
    
    //State Machine
        private enum STATE{
            INTAKE_STATE, OUTTAKE_STATE, STOW_STATE;
        }
    
    //Tracker Variables
        private boolean fuelDetectedIntake;
        private boolean maxFuelReached = false;
        private double currentSliderInches;
        private double desiredSliderInches;
        private STATE currentState;
        private STATE desiredState;

    //Data
        private ShuffleboardTab IntakeSubsystemTab = Shuffleboard.getTab("Intake Subsystem Tab");
        private GenericEntry fuelDetectedIntakeEntry;
        private GenericEntry maxFuelReachedEntry;
        private GenericEntry desiredSliderInchesEntry;
        private GenericEntry currentSliderInchesEntry;
        private GenericEntry currentStateEntry;
        private GenericEntry desiredStateEntry;
        private GenericEntry intake_slider_kP;
        private GenericEntry intake_slider_kI;
        private GenericEntry intake_slider_kD;
    
    //Sensors
        Sensors sensors = new Sensors();

    public IntakeSubsystem(){

        //Initializing Tracker Variables
            fuelDetectedIntake = false;
            currentSliderInches = IntakeSubsystemConstants.STOW_SLIDER_INCHES;
            desiredSliderInches = IntakeSubsystemConstants.STOW_SLIDER_INCHES;
            currentState = STATE.STOW_STATE;
            desiredState = STATE.STOW_STATE;
        
        //Initializing Shuffleboard Entries
            fuelDetectedIntakeEntry = IntakeSubsystemTab.add("Fuel Detected Intake", false).getEntry();
            maxFuelReachedEntry = IntakeSubsystemTab.add("Max Fuel Reached", false).getEntry();
            desiredSliderInchesEntry = IntakeSubsystemTab.add("Desired Slider Inches", 0.0).getEntry();
            currentSliderInchesEntry = IntakeSubsystemTab.add("Current Slider Inches", 0.0).getEntry();
            desiredStateEntry = IntakeSubsystemTab.add("Desired Intake State", desiredState.name()).getEntry();
            currentStateEntry = IntakeSubsystemTab.add("Current Intake State", currentState.name()).getEntry();
            intake_slider_kP = IntakeSubsystemTab.add("INTAKE SLIDER KP", sliderPID.getP()).getEntry();
            intake_slider_kI = IntakeSubsystemTab.add("INTAKE SLIDER KI", sliderPID.getI()).getEntry();
            intake_slider_kD = IntakeSubsystemTab.add("INTAKE SIDER KD", sliderPID.getD()).getEntry();

    }

    //Utility Methods
        public double getIntakeSliderInches(){
            return (intakeSliderEncoder.getPosition() - IntakeSubsystemConstants.SLIDER_OFFSET)*IntakeSubsystemConstants.ABS_ENC_2_INCHES_RATIO;
        }
    //Subsystem Methods
        public void intake(){
            if(!maxFuelReached && fuelDetectedIntake){
                intakeMotor.set(IntakeSubsystemConstants.INTAKE_SPEED);
            }
        }

        public void outtake(){
                intakeMotor.set(-IntakeSubsystemConstants.OUTTAKE_SPEED);
        }

        
        public void setDesired_Inches(double inches){
            desiredSliderInches = inches;
        }

    //Command Based methods
        public Command intakeCommand(){
            return Commands.runOnce(()->{
                desiredState = STATE.INTAKE_STATE;
            });
        }
        public Command outtakeCommand(){
            return Commands.runOnce(()->{
                desiredState = STATE.OUTTAKE_STATE;
            });
        }
        public Command stowCommand(){
            return Commands.runOnce(()->{
                desiredState = STATE.STOW_STATE;
            });
        }

    @Override
        public void periodic(){
            //Update Tracker variabls
                fuelDetectedIntake = sensors.getIntakeSensor();
                maxFuelReached = sensors.getHopperLimitSensor();
                currentSliderInches = getIntakeSliderInches();
            //STATE MACHINE
                //SET DESIRED INTAKE ANGLES
                    if(desiredState == STATE.STOW_STATE && currentState != STATE.STOW_STATE){
                        setDesired_Inches(IntakeSubsystemConstants.STOW_SLIDER_INCHES);
                        currentState = STATE.STOW_STATE;
                    } else if (desiredState == STATE.INTAKE_STATE && currentState != STATE.INTAKE_STATE){
                        setDesired_Inches(IntakeSubsystemConstants.INTAKE_SLIDER_INCHES);
                        currentState = STATE.INTAKE_STATE;
                    } else if (desiredState == STATE.OUTTAKE_STATE && currentState != STATE.OUTTAKE_STATE){
                        setDesired_Inches(IntakeSubsystemConstants.OUTTAKE_SLIDER_INCHES);
                        currentState = STATE.OUTTAKE_STATE;
                    }
                //RUN MOTOR
                    if(currentState == STATE.OUTTAKE_STATE){
                        outtake();
                    } else if (currentState == STATE.INTAKE_STATE){
                        intake();
                    } else {
                        intakeMotor.stopMotor();
                    }
            //Slider
                double sPID = sliderPID.calculate(currentSliderInches, desiredSliderInches);
                    intakeSliderMotor.setVoltage(sPID);
            //Data
                fuelDetectedIntakeEntry.setBoolean(fuelDetectedIntake);
                maxFuelReachedEntry.setBoolean(maxFuelReached);
                desiredSliderInchesEntry.setDouble(desiredSliderInches);
                currentSliderInchesEntry.setDouble(currentSliderInches);
                currentStateEntry.setString(currentState.name());
                desiredStateEntry.setString(desiredState.name());
            //Change PID Values
                sliderPID.setPID(
                    intake_slider_kP.getDouble(sliderPID.getP()), 
                    intake_slider_kI.getDouble(sliderPID.getI()), 
                    intake_slider_kD.getDouble(sliderPID.getD())
                );
        }
}
