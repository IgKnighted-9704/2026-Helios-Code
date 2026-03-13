package frc.robot.subsystems.utility;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.SubsystemConstants.LightSensor;

public class Sensors {
    public final DigitalInput indexSensorA = new DigitalInput(LightSensor.INDEXER_SENSOR_ID_A);
    public final DigitalInput indexSensorB = new DigitalInput(LightSensor.INDEXER_SENSOR_ID_B);

    public boolean getIndexSensorA() {
        return !indexSensorA.get();
    }
    public boolean getIndexSensorB(){
        return !indexSensorB.get();
    }
}
