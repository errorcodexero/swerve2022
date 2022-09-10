package frc.robot.subsystems.shooter;

import org.xero1425.base.motors.MotorController;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;

public class ShooterSubsystem extends Subsystem{
    private MotorEncoderSubsystem wheel1Subsystem_;
    private MotorEncoderSubsystem wheel2Subsystem_;
    private MotorEncoderSubsystem hoodSubsystem_;
    public static final String SubsystemName = "shooter";

    public ShooterSubsystem(Subsystem parent) throws Exception {
        super(parent, SubsystemName);

        double ticks_per_rev = 42 ;
        double seconds_per_minute = 60 ;
        double motor_to_shooter_gear_ratio  = 1.25 ;
        double factor =  seconds_per_minute / ticks_per_rev * motor_to_shooter_gear_ratio ;

        wheel1Subsystem_ = new MotorEncoderSubsystem(this, SubsystemName + "-wheel-1", false, 8);
        addChild(wheel1Subsystem_) ;
        wheel1Subsystem_.setVelocityConversion(factor);

        if (isSettingDefined("ramprate")) {
            MotorController ctrl = wheel1Subsystem_.getMotorController() ;
            double rate = getSettingsValue("ramprate").getDouble() ;
            ctrl.setOpenLoopRampRate(rate);

            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getLoggerID()).add("Settings shooter ramp rate") ;
            logger.add("rate", rate).endMessage();

            System.out.println("RAME RATE IS SET") ;
        }

        wheel2Subsystem_ = new MotorEncoderSubsystem(this, SubsystemName + "-wheel-2", false, 8);
        addChild(wheel2Subsystem_) ;
        wheel2Subsystem_.setVelocityConversion(factor);

        if (isSettingDefined("ramprate")) {
            MotorController ctrl = wheel2Subsystem_.getMotorController() ;
            double rate = getSettingsValue("ramprate").getDouble() ;
            ctrl.setOpenLoopRampRate(rate);
        }

        hoodSubsystem_ = new HoodMotorSubsystem(this) ;
        addChild(hoodSubsystem_) ;
    }

    public MotorEncoderSubsystem getWheel1Subsystem(){
        return wheel1Subsystem_;
    }

    public MotorEncoderSubsystem getWheel2Subsystem(){
        return wheel2Subsystem_;
    }

    public MotorEncoderSubsystem getHoodSubsystem() {
        return hoodSubsystem_;
    }

    public void stop() {
        wheel1Subsystem_.cancelAction();
        wheel1Subsystem_.setPower(0.0) ;
        wheel2Subsystem_.cancelAction();
        wheel2Subsystem_.setPower(0.0) ;
    }

    @Override
    public SettingsValue getProperty(String name) {
        SettingsValue v = null ;

        if (name.equals("wheel-velocity")) {
            v = new SettingsValue(wheel1Subsystem_.getVelocity()) ;
        }

        return v ;
    }
}