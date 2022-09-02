package frc.robot.subsystems.bwgconveyor;

import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.SettingsValue;

import edu.wpi.first.wpilibj.DigitalInput;

public class ConveyorSubsystem extends MotorSubsystem {
    public enum State
    {
        Idle,
        CollectFirst,
        WaitForFirstStage,
        CollectSecond,
        WaitForShooter,
        Eject
    } ;

    private DigitalInput sensor_intake_ ;
    private DigitalInput sensor_shooter_ ;
    private DigitalInput sensor_staging_ ;
    private boolean sensor_intake_value_ ;
    private boolean sensor_shooter_value_ ;
    private boolean sensor_staging_value_ ;

    private double conveyor_on_ ;
    private double conveyor_off_ ;
    private double conveyor_eject_ ;
    private double current_power_ ;

    private XeroTimer eject_ball_timer_ ;

    private State current_state_ ;
    private int ball_count_ ;
    private boolean stop_requested_ ;
    
    public ConveyorSubsystem(Subsystem subsystem) throws BadParameterTypeException, MissingParameterException {
        super(subsystem, "bwgconveyor") ;

        sensor_intake_ = new DigitalInput(getSettingsValue("sensors:intake").getInteger()) ;
        sensor_shooter_ = new DigitalInput(getSettingsValue("sensors:shooter").getInteger()) ;
        sensor_staging_ = new DigitalInput(getSettingsValue("sensors:staging").getInteger()) ;
        
        conveyor_on_ = getSettingsValue("power:on").getDouble() ;
        conveyor_off_ = getSettingsValue("power:off").getDouble() ;
        conveyor_eject_ = getSettingsValue("power:eject").getDouble() ;

        double duration = getSettingsValue("delays:first-ball").getDouble() ;

        duration = getSettingsValue("delays:eject").getDouble() ;
        eject_ball_timer_= new XeroTimer(subsystem.getRobot(), "eject_timer", duration / 1000.0) ;

        ball_count_ = 0 ;
        stop_requested_ = false ;
        current_power_ = 0.0 ;
        current_state_ = State.Idle ;
    }

    @Override
    public SettingsValue getProperty(String name) {
        SettingsValue v = null ;

        if (name.equals("ball-count")) {
            v = new SettingsValue(getBallCount()) ;
        }
        else if (name.equals("power")) {
            v = new SettingsValue(current_power_) ;
        }

        return v ;
    }

    public int getBallCount() {
        return ball_count_ ;
    }

    public boolean isFull() {
        return getBallCount() == 2 ;
    }

    public void collect() {
        if (ball_count_ == 0) {
            current_state_ = State.CollectFirst ;
        }
        else if (ball_count_ == 1) {
            current_state_ = State.CollectSecond ;
        }

        getRobot().getMessageLogger().startMessage(MessageType.Debug, getLoggerID()).add("conveyor started collect operation, ") ;
        getRobot().getMessageLogger().add("state is " + current_state_.toString()).endMessage();
    }

    public void shoot() {
    }

    public void eject() {
        current_state_ = State.Eject ;
        setConveyorPower(conveyor_eject_) ;
        eject_ball_timer_.start() ;
    }

    public void stop() {
        getRobot().getMessageLogger().startMessage(MessageType.Debug, getLoggerID()).add("conveyor stop requested").endMessage(); ;
        stop_requested_ = true ;
    }

    public boolean isIdle() {
        return current_state_ == State.Idle ;
    }

    @Override
    public void computeState() {
        boolean old_intake = sensor_intake_value_ ;
        boolean old_shooter = sensor_shooter_value_ ;
        boolean old_staging = sensor_staging_value_ ;

        sensor_intake_value_ = !sensor_intake_.get() ;
        sensor_shooter_value_ = !sensor_shooter_.get() ;
        sensor_staging_value_ = !sensor_staging_.get() ;

        putDashboard("intake", DisplayType.Always, sensor_intake_value_) ;
        putDashboard("shooter", DisplayType.Always, sensor_shooter_value_);
        putDashboard("staging", DisplayType.Always, sensor_staging_value_);

        String diff = "" ;
        if (old_intake != sensor_intake_value_) {
            diff += "intake: " + String.valueOf(old_intake) + " -> " + String.valueOf(sensor_intake_value_) ;
        }

        if (old_staging != sensor_staging_value_) {
            if (diff.length() > 0)
                diff += ", " ;
            diff += "staging: " + String.valueOf(old_staging) + " -> " + String.valueOf(sensor_staging_value_) ;
        }
        
        if (old_shooter != sensor_shooter_value_) {
            if (diff.length() > 0)
                diff += ", " ;
            diff += "shooter: " + String.valueOf(old_shooter) + " -> " + String.valueOf(sensor_shooter_value_) ;
        }

        if (diff.length() > 0)
            getRobot().getMessageLogger().startMessage(MessageType.Debug, getLoggerID()).add("sensors changed: ").add(diff).endMessage();
    }

    @Override
    public void run() throws Exception {
        super.run() ;
        
        State st = current_state_ ;

        switch(current_state_)
        {
            case Idle:
                stop_requested_ = false ;
                break ;

            case CollectFirst:
                collectFirst() ;
                break ;

            case WaitForFirstStage:
                waitForFirstStage() ;
                break ;

            case CollectSecond:
                collectSecond() ;
                break ;

            case WaitForShooter:
                waitForShooter() ;
                break ;

            case Eject:
                ejectState() ;
                break ;
        }

        if (st != current_state_) {
            getRobot().getMessageLogger().startMessage(MessageType.Debug, getLoggerID()).add("conveyor state: ") ;
            getRobot().getMessageLogger().add(st.toString()).add(" --> ").add(current_state_.toString()).endMessage();
        }
    }

    private void setConveyorPower(double p) {
        current_power_ = p ;
        super.setPower(p);
    }

    private void ejectState() {
        if (eject_ball_timer_.isExpired()) {
            ball_count_ = 0 ;
            stop_requested_ = false ;
            current_state_ = State.Idle ;
            setConveyorPower(conveyor_off_) ;
        }
    }

    private void collectFirst() {
        if (sensor_intake_value_ == true) {
            setConveyorPower(conveyor_on_) ;
            current_state_ = State.WaitForFirstStage ;
        }
    }

    private void waitForFirstStage() {
        if (sensor_staging_value_ == true && sensor_intake_value_ == true) {
            ball_count_ = 1 ;
            current_state_ = State.WaitForShooter ;
        }
        else if (sensor_staging_value_ == true) {
            ball_count_ = 1 ;
            setConveyorPower(conveyor_off_) ;
            if (stop_requested_) {
                current_state_ = State.Idle ;
                stop_requested_ = false ;
            }
            else {
                current_state_ = State.CollectSecond ;
            }
        }
    }

    private void collectSecond() {
        if (sensor_intake_value_ == true) {
            setConveyorPower(conveyor_on_) ;
            current_state_ = State.WaitForShooter ;
        }
    }

    private void waitForShooter() {
        if (sensor_shooter_value_ == true) {
            ball_count_  = 2 ;
            setConveyorPower(conveyor_off_) ;
            stop_requested_ = false ;
            current_state_ = State.Idle ;
        }
    }
}