package org.xero1425.base.motorsubsystem;

import org.xero1425.base.Subsystem;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.Speedometer;

/// \file

/// \brief A subsystem that includes a single motor, or group of motors mechanically coupled and an encoder for closed loop control
public class MotorEncoderSubsystem extends MotorSubsystem
{
    // The speedometer measuring the speed of the motor output
    private Speedometer speedometer_ ;

    // The encoder attached to the motor output
    private XeroEncoder encoder_ ;

    // If true, the measured output is angular
    private boolean angular_ ;

    // If true, use the velocity from the motor controller
    private boolean use_ctrl_velocity_ ;

    /// \brief Create the subsystem
    /// \param parent the owning subsystem
    /// \param name the name of this subsystem
    /// \param angle if true, the measured output is angular
    public MotorEncoderSubsystem(Subsystem parent, String name, boolean angle) throws Exception {
        super(parent, name) ;

        speedometer_ = new Speedometer(name, 2, angle) ;
        angular_ = angle ;

        use_ctrl_velocity_ = false ;

        String encname = "subsystems:" + name + ":hw:encoder" ;
        encoder_ = new XeroEncoder(parent.getRobot(), encname, angle, getMotorController()) ;
    }

    /// \brief Create the subsystem
    /// \param parent the owning subsystem
    /// \param name the name of this subsystem
    /// \param angle if true, the measured output is angular
    /// \param samples the number of samples to average for output position and velocity
    public MotorEncoderSubsystem(Subsystem parent, String name, boolean angle, int samples) throws Exception {
        super(parent, name) ;

        speedometer_ = new Speedometer(name, samples, angle) ;
        angular_ = angle ;

        String encname = "subsystems:" + name + ":hw:encoder" ;
        encoder_ = new XeroEncoder(parent.getRobot(), encname, angle, getMotorController()) ;
    }

    /// \brief Returns true if the motor has a hardware PID loop in the motor controller
    /// \returns true if the motor has a hardware PID loop in the motor controller
    public boolean hasHWPID() {
        boolean ret = false ;

        try {
            ret = getMotorController().hasPID() ;
        }
        catch(Exception ex) {
            ret = false ;
        }

        return ret;
    }

    /// \brief Set the position conversion factor in the motor controller
    /// This only applies when running the PID loop on the motor controller
    /// \param factor the factor to multiple by the encoder ticks to get real world position
    public void setPositionConversion(double factor) throws BadMotorRequestException, MotorRequestFailedException {
        getMotorController().setPositionConversion(factor);
    }

    /// \brief Set the velocity conversion factor in the motor controller
    /// This only applies when running the PID loop on the motor controller    
    /// \param factor the factor to multiply the encoder ticks rate by to get real world velocity
    public void setVelocityConversion(double factor) throws BadMotorRequestException, MotorRequestFailedException {
        getMotorController().setVelocityConversion(factor);
        use_ctrl_velocity_ = true ;
    }

    /// \brief Returns true if measuring an angular quantity
    /// \returns true if measuring an angular quantity
    public boolean isAngular() {
        return angular_ ;
    }

    /// \brief Returns the position of the motor output, as measured by the speedometer
    /// \returns the position of the motor output, as measured by the speedometer
    public double getPosition() {
        return speedometer_.getDistance() ;
    }

    /// \brief Returns the velocity of the motor output, as measured by the speedometer
    /// \returns the velocity of the motor output, as measured by the speedometer    
    public double getVelocity() {
        double ret = 0.0 ;

        if (use_ctrl_velocity_) {
            try {
                ret = getMotorController().getVelocity() ;
            } catch (BadMotorRequestException | MotorRequestFailedException e) {
                ret = 0.0 ;
            }
        }
        else {
            ret = speedometer_.getVelocity() ;
        }

        return ret ;
    }

    /// \brief Returns the acceleration of the motor output, as measured by the speedometer
    /// \returns the acceleration of the motor output, as measured by the speedometer     
    public double getAcceleration() {
        return speedometer_.getAcceleration() ;
    }

    /// \brief Calibrates the motor encoder with the given position
    /// \param pos the current real world position of the motor output
    public void calibrate(double pos) {
        encoder_.calibrate(pos) ;
    }

    /// \brief Reset the motor and attached encoder.  This will reset the encoder value to 
    /// zero and set the motor power to off.
    public void reset() {
        super.reset() ;
        encoder_.reset() ;
    }

    /// \brief Returns the encoder raw count
    /// \returns the encoder raw count
    public double getEncoderRawCount() {
        return encoder_.getRawCount() ;
    }

    /// \brief Called once per robot loop by the Xero Framework to compute the position, velocity, and
    /// acceleration of the motor in real world units.  It also displays the position and velocity on the
    /// SmartDashboard if verbose output is enabled for this subsystem.
    @Override
    public void computeMyState() throws Exception {
        super.computeMyState();


        double pos = encoder_.getPosition() ;
        speedometer_.update(getRobot().getDeltaTime(), pos) ;

        MessageLogger logger = getRobot().getMessageLogger()  ;
        logger.startMessage(MessageType.Debug, getLoggerID()) ;
        logger.add(getName()) ;
        logger.add("power", getPower()) ;
        logger.add("pos", pos) ;
        logger.add("velocity", speedometer_.getVelocity());
        logger.add("accel", speedometer_.getAcceleration()) ;
        logger.add("ticks", getEncoderRawCount()) ;
        logger.endMessage();
    }
}