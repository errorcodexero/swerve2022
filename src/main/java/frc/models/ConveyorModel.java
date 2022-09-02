package frc.models;

import edu.wpi.first.hal.simulation.DIODataJNI;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.SettingsValue;

public class ConveyorModel extends SimulationModel {
    
    private int intake_sensor_io_ ;
    private int shooter_sensor_io_ ;
    private int staging_sensor_io_ ;

    public ConveyorModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst);
    }

    @Override
    public void endCycle() {
    }

    public boolean create() {
        try {
            intake_sensor_io_ = getProperty("intake-sensor").getInteger() ;
            DIODataJNI.setIsInput(intake_sensor_io_, true) ;
            DIODataJNI.setValue(intake_sensor_io_, true) ;

            shooter_sensor_io_ = getProperty("shooter-sensor").getInteger() ;
            DIODataJNI.setIsInput(shooter_sensor_io_, true) ;
            DIODataJNI.setValue(shooter_sensor_io_, true) ;

            staging_sensor_io_ = getProperty("middle-sensor").getInteger() ;
            DIODataJNI.setIsInput(staging_sensor_io_, true) ;
            DIODataJNI.setValue(staging_sensor_io_, true) ;
        }
        catch(BadParameterTypeException ex) {
            return false ;
        }

        setCreated();
        return true ;
    }

    public boolean processEvent(String name, SettingsValue value) {
        try {
            if (name.equals("intake")) {
                if (value.isBoolean()) {
                    DIODataJNI.setValue(intake_sensor_io_, value.getBoolean()) ;
                }
            }
            else if (name.equals("shooter")) {
                if (value.isBoolean()) {
                    DIODataJNI.setValue(shooter_sensor_io_, value.getBoolean()) ;
                }
            }    
            else if (name.equals("middle")) {
                if (value.isBoolean()) {
                    DIODataJNI.setValue(staging_sensor_io_, value.getBoolean()) ;
                }
            }            
        }
        catch(BadParameterTypeException ex) {
            //
            // Should never happen, we check the types before getting the value
            //
        }
        return true ;
    }

    public void run(double dt) {
    }
}