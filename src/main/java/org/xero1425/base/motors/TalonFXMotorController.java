package org.xero1425.base.motors;

import com.ctre.phoenix.ErrorCode;

/// \file
/// This file conatins the implementation of the CTREMotorController class.  This class
/// is derived from the MotorController class and supports the CTRE devices including the TalonFX,
/// the TalonSRX, and the VictorSPX.
///

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.RobotBase;

/// \brief This class is MotorController class that supports the TalonFX motors.   This class is a 
/// wrapper for the TalonFX class that provides the interface that meets the requirements of the 
/// MotorController base class.
public class TalonFXMotorController extends MotorController
{  
    private TalonFX controller_ ;
    private boolean inverted_ ;
    private boolean pid_setup_ ;

    private SimDevice sim_ ;
    private SimDouble sim_power_ ;
    private SimDouble sim_encoder_ ;
    private SimBoolean sim_motor_inverted_ ;
    private SimBoolean sim_neutral_mode_ ;

    /// \brief the name of the device when simulating
    public final static String SimDeviceName = "CTREMotorController" ;

    private final int ControllerTimeout = 250 ;

    /// \brief Create a new TalonFX Motor Controller.
    /// \param name the name of this motor
    /// \param index the CAN address of this motor controller
    public TalonFXMotorController(String name, int index) throws MotorRequestFailedException {
        super(name) ;

        inverted_ = false ;
        pid_setup_ = false ;

        if (RobotBase.isSimulation()) {
            sim_ = SimDevice.create(SimDeviceName, index) ;

            //
            // Create a simulated motor that can be accessed by simulation models
            //
            sim_power_ = sim_.createDouble(MotorController.SimPowerParamName, SimDevice.Direction.kBidir, 0.0) ;
            sim_encoder_ = sim_.createDouble(MotorController.SimEncoderParamName, SimDevice.Direction.kBidir, 0.0) ;
            sim_motor_inverted_ = sim_.createBoolean(MotorController.SimInvertedParamName, SimDevice.Direction.kBidir, false) ;
            sim_neutral_mode_ = sim_.createBoolean(MotorController.SimNeutralParamName, SimDevice.Direction.kBidir, false) ;
            sim_.createBoolean(MotorController.SimEncoderStoresTicksParamName, SimDevice.Direction.kBidir, true) ;
        }
        else {


            sim_ = null ;
            sim_power_ = null ;
            sim_encoder_ = null ;

            controller_ = new TalonFX(index) ;

            controller_.configVoltageCompSaturation(12.0, ControllerTimeout) ;
            controller_.enableVoltageCompensation(true);
        }
    }

    public double getVelocity() throws BadMotorRequestException, MotorRequestFailedException {
        return 0.0 ;
    }

    /// \brief Return the current input voltage to the motor controller
    /// \returns the current input voltage to the motor controller
    public double getInputVoltage() throws BadMotorRequestException {
        return controller_.getBusVoltage() ;
    }

    /// \brief Return the motor voltage applied to the motor
    /// \returns the motor voltage applied to the motor    
    public double getAppliedVoltage() throws BadMotorRequestException {
        return controller_.getMotorOutputVoltage() ;
    }
    
    /// \brief Returns true if the motor controller supports PID loops on the controller
    /// \returns true if the motor controller supports PID loops on the controller
    public boolean hasPID() throws BadMotorRequestException {
        return false ;
    }

    /// \brief Set the target if running a PID loop on the motor controller
    /// \param target the target for the PID loop on the motor controller    
    public void setTarget(double target) throws BadMotorRequestException {
        if (pid_setup_ == false)
            throw new BadMotorRequestException(this, "calling setTarget() before calling setPID()");

        TalonFX ctrl = (TalonFX)controller_ ;
        ctrl.set(TalonFXControlMode.Velocity, target) ;
    }

    /// \brief Set the PID parameters for a PID loop running on the motor controller
    /// \param type the type of pid loop (velocity or position)
    /// \param p the proportional parameter for the PID controller
    /// \param i the integral parameter for the PID controller
    /// \param d the derivative parameter for the PID controller
    /// \param f the feed forward parameter for the PID controller
    /// \param outmax the maximum output parameter for the PID controller 
    public void setPID(PidType type, double p, double i, double d, double f, double outmax) throws BadMotorRequestException, MotorRequestFailedException {
        ErrorCode code ;

        code = controller_.config_kP(0, p, ControllerTimeout) ;
        if (code != ErrorCode.OK)
            throw new MotorRequestFailedException(this, "CTRE config_kP() call failed during setPID() call", code) ; 

        code = controller_.config_kI(0, i, ControllerTimeout) ;
        if (code != ErrorCode.OK)
            throw new MotorRequestFailedException(this, "CTRE config_kI() call failed during setPID() call", code) ; 

        code = controller_.config_kD(0, d, ControllerTimeout) ;
        if (code != ErrorCode.OK)
            throw new MotorRequestFailedException(this, "CTRE config_kD() call failed during setPID() call", code) ; 

        code = controller_.config_kF(0, f, ControllerTimeout) ;
        if (code != ErrorCode.OK)
            throw new MotorRequestFailedException(this, "CTRE config_kF() call failed during setPID() call", code) ;  

        code = controller_.configClosedLoopPeakOutput(0, outmax, ControllerTimeout) ;
        if (code != ErrorCode.OK)
            throw new MotorRequestFailedException(this, "CTRE config_kF() call failed during setPID() call", code) ; 

        pid_setup_= true ;
    }

    /// \brief Stop the PID loop in the motor controller    
    public void stopPID() throws BadMotorRequestException {
        controller_.set(ControlMode.PercentOutput, 0.0) ;
    }

    /// \brief Set the factor for converting encoder units to real world units, only applies to the PID loop on the motor controller
    /// \param factor the factor to convert encoder units to real world units    
    public void setPositionConversion(double factor) throws BadMotorRequestException, MotorRequestFailedException {
        ErrorCode code = controller_.configSelectedFeedbackCoefficient(factor, 0, ControllerTimeout) ;
        if (code != ErrorCode.OK)
            throw new MotorRequestFailedException(this, "CTRE configSelectedFeedbackCoefficient() call failed during setPositionConversion() calls", code) ;         
    }

    /// \brief Set the factor for converting encoder units to real world units, only applies to the PID loop on the motor controller
    /// \param factor the factor to convert encoder units to real world units     
    public void setVelocityConversion(double factor) throws BadMotorRequestException, MotorRequestFailedException {
        ErrorCode code = controller_.configSelectedFeedbackCoefficient(factor, 0, ControllerTimeout) ;
        if (code != ErrorCode.OK)
            throw new MotorRequestFailedException(this, "CTRE configSelectedFeedbackCoefficient() call failed during setPositionConversion() calls", code) ; 
    }

    /// \brief Set the motor power
    /// \param percent the motor power to assign to the motor    
    public void set(double percent) {
        if (sim_ != null) {
            sim_power_.set(percent) ;
        }
        else {
            controller_.set(ControlMode.PercentOutput, percent) ;
        }
    }

    /// \brief Set the motor to invert the direction of motion 
    /// \param inverted if true invert the direction of motion, otherwise do not    
    public void setInverted(boolean inverted) {
        if (sim_ != null) {
            sim_motor_inverted_.set(true) ;
        }
        else {
            controller_.setInverted(inverted);
        }
        inverted_ = inverted ;
    }

    /// \brief Returns true if the motor is inverted
    /// \returns true if the motor is inverted    
    public boolean isInverted() {
        return inverted_ ;
    }

    /// \brief Reapplies the inverted status of the motor.  When setInverted() is called, the inverted state of the motor
    /// is stored and this method reapplies that stored state to the motor controller.  This was put into place because some
    /// motors setup to follow other motors lost their inverted state when the robot was disabled and re-enabled.
    public void reapplyInverted() {
        if (sim_ != null) {
            sim_motor_inverted_.set(inverted_) ;
        }
        else {
            controller_.setInverted(inverted_);
        }
    }

    /// \brief Set the neutral mode for the motor
    /// \param mode the neutral mode for the motor    
    public void setNeutralMode(NeutralMode mode) throws BadMotorRequestException {
        if (sim_ != null) {
            switch(mode)
            {
                case Coast:
                    sim_neutral_mode_.set(false) ;
                    break ;

                case Brake:
                    sim_neutral_mode_.set(true) ;
                break ;            
            }
        }
        else {
            switch(mode)
            {
                case Coast:
                    controller_.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
                    break ;

                case Brake:
                    controller_.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
                break ;            
            }
        }
    }

    /// \brief Set the current motor to follow another motor.  Note the motors must be compatible with each other for following.
    /// \param ctrl the other motor to follow
    /// \param invert if true, follow the other motor but with the power inverted.
    /// \throws MotorRequestFailedException if the motors are not compatible for following.    
    public void follow(MotorController ctrl, boolean invert) throws BadMotorRequestException {
        if (sim_ == null) {
            if (invert)
                throw new BadMotorRequestException(this, "cannot follow another controller inverted") ;

            try {
                TalonFXMotorController other = (TalonFXMotorController)ctrl ;
                controller_.follow(other.controller_) ;
            }
            catch(ClassCastException ex)
            {
                throw new BadMotorRequestException(this, "cannot follow a motor that is of another type") ;
            }
        }
    }

    /// \brief Return a human readable string giving the physical motor controller type
    /// \returns a human readable string giving the physical motor controller type    
    public String getType() {
        return "TalonFX" ;
    }

    /// \brief Returns true if the motor encoder has an embedded encoder that can return position
    /// \returns true if the motor encoder has an embedded encoder that can return position    
    public boolean hasPosition() {
        return true ;
    }

    /// \brief Returns the position of the motor in motor units.  If the setPositionConversion() has been called
    /// then these units will be based on the factor supplied.  Otherwise these units are in encoder ticks.
    /// \returns the position of the motor in motor units    
    public double getPosition() throws BadMotorRequestException {
        double ret = 0 ;

        if (sim_ != null) {
            ret = (int)sim_encoder_.getValue().getDouble() ;
        }
        else {
            TalonFX fx = (TalonFX)controller_ ;
            ret = fx.getSelectedSensorPosition() ;
        }
        
        return ret ;
    }

    /// \brief Reset the encoder values to zero    
    public void resetEncoder() throws BadMotorRequestException {
        if (sim_ != null) {
            sim_encoder_.set(0.0) ;
        }
        else {
            TalonFX fx = (TalonFX)controller_ ;
            fx.setSelectedSensorPosition(0) ;
        }
    }

    /// \brief Set the current limit for the current supplied to the motor
    /// \param limit the amount of current, in amps,  to the value given    
    public void setCurrentLimit(double limit) throws BadMotorRequestException {
        if (sim_ == null) {
            TalonFX fx = (TalonFX)controller_ ;
            SupplyCurrentLimitConfiguration cfg = new SupplyCurrentLimitConfiguration(true, limit, limit, 1) ;
            fx.configSupplyCurrentLimit(cfg) ;
        }
    }     
    
    /// \brief Set the open loop ramp rate for the motor
    /// \param limit the amount of time for the motor to ramp from no power to full power    
    public void setOpenLoopRampRate(double limit) throws BadMotorRequestException {
        if (sim_ == null) {
            TalonFX fx = (TalonFX)controller_ ;
            fx.configOpenloopRamp(limit, 20) ;
        }
    }  

    /// \brief Return the firmware version of the motor controller
    /// \returns the firmware version of the motor controller    
    public String getFirmwareVersion() throws BadMotorRequestException {
        int v = controller_.getFirmwareVersion() ;

        return String.valueOf((v >> 8) & 0xff) + "." + String.valueOf(v & 0xff) ;
    }

    /// \brief Set the encoder update frequency.  This configures the rate at which the motor controller
    /// sends back the CAN status packets that contain encoder information form the motor controller to 
    /// the software running on the RoboRio.
    /// \param freq the frequency to update the encoder values    
    public void setEncoderUpdateFrequncy(EncoderUpdateFrequency freq) throws BadMotorRequestException {
        if (controller_ != null)
        {
            if (freq == EncoderUpdateFrequency.Infrequent) {
                controller_.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 500) ;
                controller_.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 500) ;
            }
            else if (freq == EncoderUpdateFrequency.Default) {
                controller_.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10) ;
                controller_.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20) ;            
            }
            else if (freq == EncoderUpdateFrequency.Frequent) {
                controller_.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10) ;
                controller_.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10) ;             
            }
        }        
    }     
} ;
