package org.xero1425.simulator.models;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveDriveModel extends SimulationModel {

    static final String SwerveDriveXPos = "xpos";
    static final String SwerveDriveYPos = "ypos";
    static final String SwerveDriveAngle = "angle";

    static public final int FL = 0; // Index of the front left module
    static public final int FR = 1; // Index of the front right module
    static public final int BL = 2; // Index of the back left module
    static public final int BR = 3; // Index of the back right module

    private Pose2d pose_;

    private TurrentSubModel[] steer_;
    private WheelSubModel[] drive_;
    private NavXModel navx_ ;
    private double meters_per_sec_per_power_ ;
    private double degs_per_sec_per_power_ ;

    private double width_ ;
    private double length_ ;
    private double ticks_per_meter_ ;

    private SwerveDriveKinematics kinematics_ ;
    private SwerveDriveOdometry odometry_ ;

    public SwerveDriveModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst);

        pose_ = new Pose2d();
        steer_ = new TurrentSubModel[4];
        drive_ = new WheelSubModel[4];
    }

    public Pose2d getPose() {
        return pose_ ;
    }

    public boolean create() {
        int encoder ;
        SimMotorController motor ;

        try {
            width_ = getProperty("width").getDouble() ;
            length_ = getProperty("length").getDouble() ;
            meters_per_sec_per_power_ = getProperty("meters-per-second-per-power").getDouble() ;
            degs_per_sec_per_power_ = getProperty("degrees-per-second-per-power").getDouble() ;
            ticks_per_meter_ = getProperty("ticks-per-meter").getDouble() ;
        }
        catch(Exception ex) {
            return false ;
        }

        
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
        
        Translation2d fl = new Translation2d(width_ / 2.0, length_ / 2.0) ;
        Translation2d fr = new Translation2d(width_ / 2.0, -length_ / 2.0) ;
        Translation2d bl = new Translation2d(-width_ / 2.0, length_ / 2.0) ;
        Translation2d br = new Translation2d(-width_ / 2.0, -length_ / 2.0) ;

        kinematics_ = new SwerveDriveKinematics(fl, fr, bl, br) ;
        odometry_ = new SwerveDriveOdometry(kinematics_, pose_.getRotation()) ;

        try {
            encoder = getIntProperty("fl-steer:encoder");
            ec = getDoubleProperty("fl-steer:ec") ;
        }
        catch(Exception ex) {
            return false ;
        }
        motor = new SimMotorController(this, "fl-steer");
        if (!motor.createMotor())
            return false;
        steer_[FL] = new TurrentSubModel(motor, degs_per_sec_per_power_, encoder, rmax, rmin, emax, emin, rc, ec) ;

        motor = new SimMotorController(this, "fl-drive");
        if (!motor.createMotor())
            return false;
        drive_[FL] = new WheelSubModel(motor, ticks_per_meter_, meters_per_sec_per_power_) ;

        try {
            encoder = getIntProperty("fr-steer:encoder");
            ec = getDoubleProperty("fr-steer:ec") ;            
        }
        catch(Exception ex) {
            return false ;
        }        
        motor = new SimMotorController(this, "fr-steer");
        if (!motor.createMotor())
            return false;
        steer_[FR] = new TurrentSubModel(motor, degs_per_sec_per_power_, encoder, rmax, rmin, emax, emin, rc, ec) ;

        motor = new SimMotorController(this, "fr-drive");
        if (!motor.createMotor())
            return false;
        drive_[FR] = new WheelSubModel(motor, ticks_per_meter_, meters_per_sec_per_power_) ;

        try {
            encoder = getIntProperty("bl-steer:encoder");
            ec = getDoubleProperty("bl-steer:ec") ;            
        }
        catch(Exception ex) {
            return false ;
        }           
        motor = new SimMotorController(this, "bl-steer");
        if (!motor.createMotor())
            return false;
        steer_[BL] = new TurrentSubModel(motor, degs_per_sec_per_power_, encoder, rmax, rmin, emax, emin, rc, ec) ;

        motor = new SimMotorController(this, "bl-drive");
        if (!motor.createMotor())
            return false;
        drive_[BL] = new WheelSubModel(motor, ticks_per_meter_, meters_per_sec_per_power_) ;

        try {
            encoder = getIntProperty("br-steer:encoder");
            ec = getDoubleProperty("br-steer:ec") ;
        }
        catch(Exception ex) {
            return false ;
        }         
        motor = new SimMotorController(this, "br-steer");
        if (!motor.createMotor())
            return false;
        steer_[BR] = new TurrentSubModel(motor, degs_per_sec_per_power_, encoder, rmax, rmin, emax, emin, rc, ec) ;

        motor = new SimMotorController(this, "br-drive");
        if (!motor.createMotor())
            return false;
        drive_[BR] = new WheelSubModel(motor, ticks_per_meter_, meters_per_sec_per_power_) ;

        //
        // Attach to the navx model to update the navx angle settings as the robot turns
        //
        if (hasProperty("navx:model") && getProperty("navx:model").isString() && hasProperty("navx:instance") && getProperty("navx:instance").isString()) {

            String navx_model = null ;
            String navx_inst = null ;

            try {
                navx_model = getProperty("navx:model").getString();
                navx_inst = getProperty("navx:instance").getString() ;                
            } catch (BadParameterTypeException e) {
            }

            SimulationModel model = getEngine().findModel(navx_model, navx_inst) ;
            if (model != null && (model instanceof NavXModel))
                navx_ = (NavXModel)model ;
        }            

        setCreated();
        return true;
    }

    public boolean processEvent(String name, SettingsValue value) {
        if (name.equals(SwerveDriveXPos)) {
            if (!value.isDouble()) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a double").endMessage();
                return true;
            }

            try {
                pose_ = new Pose2d(value.getDouble(), pose_.getTranslation().getY(), pose_.getRotation());
            } catch (BadParameterTypeException e) {
            }
        } else if (name.equals(SwerveDriveYPos)) {
            if (!value.isDouble()) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a double").endMessage();
                return true;
            }

            try {
                pose_ = new Pose2d(pose_.getTranslation().getX(), value.getDouble(), pose_.getRotation());
            } catch (BadParameterTypeException e) {
            }
        } else if (name.equals(SwerveDriveAngle)) {
            if (!value.isDouble()) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a double").endMessage();
                return true;
            }

            try {
                pose_ = new Pose2d(pose_.getTranslation().getX(), pose_.getTranslation().getY(),
                        Rotation2d.fromDegrees(value.getDouble()));

            } catch (BadParameterTypeException e) {
            }
        }

        odometry_.resetPosition(pose_, Rotation2d.fromDegrees(navx_.getYaw())) ;
        return true;
    }

    public void run(double dt) {
        SwerveModuleState [] states = new SwerveModuleState[4] ;

        steer_[FL].run(dt) ;        
        steer_[FR].run(dt) ;        
        steer_[BL].run(dt) ;        
        steer_[BR].run(dt) ;

        drive_[FL].run(dt) ;
        drive_[FR].run(dt) ;
        drive_[BL].run(dt) ;
        drive_[BR].run(dt) ;

        states[FL] = new SwerveModuleState(drive_[FL].getSpeed(), steer_[FL].getAngleRot()) ;
        states[FR] = new SwerveModuleState(drive_[FR].getSpeed(), steer_[FR].getAngleRot()) ;
        states[BL] = new SwerveModuleState(drive_[BL].getSpeed(), steer_[BL].getAngleRot()) ;
        states[BR] = new SwerveModuleState(drive_[BR].getSpeed(), steer_[BR].getAngleRot()) ;

        odometry_.update(Rotation2d.fromDegrees(navx_.getYaw()), states) ;
    }
}
