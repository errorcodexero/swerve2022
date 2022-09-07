package frc.models;

import edu.wpi.first.hal.simulation.AnalogInDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;
import org.xero1425.simulator.models.SimMotorController;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.EncoderMapper;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;

public class HoodModel extends SimulationModel {
        
    private SimMotorController motor_ ;
    private double degrees_per_second_per_volt_ ;
    private double angle_  ;
    private int encoder_input_ ;
    private EncoderMapper mapper_ ;
    private double voltage_ ;

    public HoodModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst);
        
        mapper_ = null ;
    }

    public boolean create() {
        motor_ = new SimMotorController(this, "hood") ;
        if (!motor_.createMotor())
            return false ;

        try {
            encoder_input_ = getProperty("encoder").getInteger() ;
        } catch (BadParameterTypeException e) {
            MessageLogger logger = getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("cannot create model ").addQuoted(getModelName()).add(" instance ").addQuoted(getInstanceName()) ;
            logger.add(" - missing parameter ").addQuoted("encoder").endMessage();
            return false ;
        }

        try {
            degrees_per_second_per_volt_ = getProperty("degrees_per_second_per_volt").getDouble();
        } catch (BadParameterTypeException e) {
            MessageLogger logger = getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("cannot create model ").addQuoted(getModelName()).add(" instance ").addQuoted(getInstanceName()) ;
            logger.add(" - missing parameter ").addQuoted("ticks_per_second_per_volt").endMessage();
            return false ;
        }

        angle_ = 0 ;

        double rmax, rmin, emax, emin, rc, ec ;

        try {
            rmax = getProperty("rmax").getDouble();
        } catch (BadParameterTypeException e) {
            MessageLogger logger = getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("cannot create model ").addQuoted(getModelName()).add(" instance ").addQuoted(getInstanceName()) ;
            logger.add(" - missing parameter ").addQuoted("rmax").endMessage();
            return false ;
        }

        try {
            rmin = getProperty("rmin").getDouble();
        } catch (BadParameterTypeException e) {
            MessageLogger logger = getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("cannot create model ").addQuoted(getModelName()).add(" instance ").addQuoted(getInstanceName()) ;
            logger.add(" - missing parameter ").addQuoted("rmin").endMessage();
            return false ;
        }   
            
        try {
            emax = getProperty("emax").getDouble();
        } catch (BadParameterTypeException e) {
            MessageLogger logger = getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("cannot create model ").addQuoted(getModelName()).add(" instance ").addQuoted(getInstanceName()) ;
            logger.add(" - missing parameter ").addQuoted("emax").endMessage();
            return false ;
        }
        
        try {
            emin = getProperty("emin").getDouble();
        } catch (BadParameterTypeException e) {
            MessageLogger logger = getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("cannot create model ").addQuoted(getModelName()).add(" instance ").addQuoted(getInstanceName()) ;
            logger.add(" - missing parameter ").addQuoted("emin").endMessage();
            return false ;
        }
        
        try {
            rc = getProperty("rc").getDouble();
        } catch (BadParameterTypeException e) {
            MessageLogger logger = getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("cannot create model ").addQuoted(getModelName()).add(" instance ").addQuoted(getInstanceName()) ;
            logger.add(" - missing parameter ").addQuoted("rc").endMessage();
            return false ;
        }
        
        try {
            ec = getProperty("ec").getDouble();
        } catch (BadParameterTypeException e) {
            MessageLogger logger = getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("cannot create model ").addQuoted(getModelName()).add(" instance ").addQuoted(getInstanceName()) ;
            logger.add(" - missing parameter ").addQuoted("ec").endMessage();
            return false ;
        }        

        mapper_ = new EncoderMapper(rmax, rmin, emax, emin) ;
        mapper_.calibrate(rc, ec);

        setCreated();
        return true ;
    }

    public boolean processEvent(String name, SettingsValue value) {
        boolean ret = false ;

        if (name.equals("angle")) {
            if (!value.isDouble()) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a double").endMessage();
                return true ;
            }

            try {
                angle_ = value.getDouble();
            } catch (BadParameterTypeException e) {
            }
        }
        return ret ;
    }

    public void run(double dt) {
        double power = motor_.getPower() ;
        angle_ += degrees_per_second_per_volt_ * dt * power ;
        voltage_ = mapper_.toEncoder(angle_) ;
        AnalogInDataJNI.setVoltage(encoder_input_, voltage_) ;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(angle_) ;
    }
}