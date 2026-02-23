package frc.robot.subsystems.misc;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants.HopperSubsystemConstants;
import frc.robot.Constants.SubsystemConstants.ShooterSubsystemConstants;
import frc.robot.subsystems.utility.Sensors;

public class HopperSubsystem extends SubsystemBase{

    //Hopper Motors
        private final SparkMax hopperMotorA = new SparkMax( HopperSubsystemConstants.HOPPER_ID_A, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        private final SparkMax hopperMotorB = new SparkMax( HopperSubsystemConstants.HOPPER_ID_B, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        private final SparkMax hopperMotorC = new SparkMax( HopperSubsystemConstants.HOPPER_ID_C, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    
    //Index Motor
        private final TalonFX indexMotor = new TalonFX(HopperSubsystemConstants.INDEX_MOTOR_ID);

    // STATE MACHINES
        private enum STATE {
            AUTO, MANUAL
        }
    //Tracker Variables
        private boolean fuelDetectedIndexer;
        private STATE indexState;
        private boolean indexManually;
        private boolean desiredVelReached;
        private boolean desiredAngleReached;
    
    //Data
        private ShuffleboardTab HopperSubsystemTab = Shuffleboard.getTab("Hopper Subsystem Tab");
        private GenericEntry fuelDetectedIndexerEntry;
        private GenericEntry indexStateEntry;
        private GenericEntry desiredVelReachedEntry;
        private GenericEntry desiredAngleReachedEntry;
    
    //Sensors 
        Sensors sensors = new Sensors();
    
    public HopperSubsystem(){

        //Tracker Variables
            fuelDetectedIndexer = false;
            indexState = STATE.AUTO;
            desiredVelReached = ShooterSubsystemConstants.desiredVelReached;
            desiredAngleReached = ShooterSubsystemConstants.desiredAngleReached;
    
        //Data
            fuelDetectedIndexerEntry = HopperSubsystemTab.add("Fuel Detected Indexer", false).getEntry();
            indexStateEntry = HopperSubsystemTab.add("Current Index State", indexState.name()).getEntry();
            desiredAngleReachedEntry = HopperSubsystemTab.add("Desired Angle Reached", desiredAngleReached).getEntry();
            desiredVelReachedEntry = HopperSubsystemTab.add("desiredVelocityReached", desiredVelReached).getEntry();
    }

    public void indexFuel(boolean runIndex){
        hopperMotorA.set(HopperSubsystemConstants.HOPPER_SPEED);
        hopperMotorB.set(HopperSubsystemConstants.HOPPER_SPEED);
        hopperMotorC.set(HopperSubsystemConstants.HOPPER_SPEED);
        indexMotor.set(HopperSubsystemConstants.INDEXER_SPEED);
    }
    
    public void stopIndex(){
        hopperMotorA.set(0);
        hopperMotorB.set(0);
        hopperMotorC.set(0);
        indexMotor.set(0);
    }

    public Command setIndexState(STATE state){
        return Commands.runOnce(()->{
            this.indexState = state;
            if(indexState == STATE.MANUAL){
                indexManually = true;
            }
        });
    }

    @Override
    public void periodic(){
        //Update Tracker Variables
            fuelDetectedIndexer = sensors.getIndexSensor();
            indexState = STATE.AUTO;
            desiredVelReached = ShooterSubsystemConstants.desiredVelReached;
            desiredAngleReached = ShooterSubsystemConstants.desiredAngleReached;
        //Data
            fuelDetectedIndexerEntry.setBoolean(fuelDetectedIndexer);
            indexStateEntry.setString(indexState.name());
            desiredVelReachedEntry.setBoolean(desiredVelReached);
            desiredAngleReachedEntry.setBoolean(desiredAngleReached);

        //Index Based On State Input
        if(indexState == STATE.AUTO){
            if(fuelDetectedIndexer && desiredVelReached && desiredAngleReached){
                indexFuel(true);
            } else {
                stopIndex();
            }
        } else if (indexState == STATE.MANUAL) {
            if(fuelDetectedIndexer){
                indexFuel(indexManually);
            }
        }
    }

}
