package frc.robot.subsystems.conveyor;

import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.SettingsValue;

import edu.wpi.first.wpilibj.DigitalInput;

public class ConveyorSubsystem extends MotorSubsystem {

    private enum State {
        Idle,
        WaitForIntake,
        WaitForMiddle,
        WaitForShooter,
        WaitForIntake2,
        Eject,
        StartShoot,
        Shooting,
        PreloadBallWaitForMiddleLow,
        PreloadBallWaitForMiddleHigh,
    }

    private State state_;

    private DigitalInput intake_sensor_;
    private DigitalInput shooter_sensor_;
    private DigitalInput middle_sensor_;

    private boolean intake_value_ = false;
    private boolean middle_value_ = false;
    private boolean shooter_value_ = false;

    private double collect_power_;
    private double off_power_;
    private double eject_power_;
    private double shoot_power_ ;
    private double preload_power_ ;

    private int ball_count_;

    private XeroTimer eject_timer_;
    private XeroTimer shoot_timer_ ;

    private boolean isStop; 

    public ConveyorSubsystem(Subsystem parent) throws BadParameterTypeException, MissingParameterException, BadMotorRequestException {
        super(parent, "conveyor");

        collect_power_ = getSettingsValue("power:collect").getDouble();
        off_power_ = getSettingsValue("power:off").getDouble();
        eject_power_ = getSettingsValue("power:eject").getDouble();
        shoot_power_ = getSettingsValue("power:shoot").getDouble() ;
        preload_power_ = getSettingsValue("power:preload").getDouble() ;

        int channel = getSettingsValue("sensors:intake").getInteger() ;
        intake_sensor_ = new DigitalInput(channel) ;

        int channel1 = getSettingsValue("sensors:middle").getInteger() ;
        middle_sensor_ = new DigitalInput(channel1) ;

        int channel2 = getSettingsValue("sensors:shooter").getInteger() ;
        shooter_sensor_ = new DigitalInput(channel2) ;

        eject_timer_ = new XeroTimer(getRobot(), "Eject", getSettingsValue("duration:eject").getDouble());
        shoot_timer_ = new XeroTimer(getRobot(), "Shoot", getSettingsValue("duration:shoot").getDouble());

        isStop = false; 

        // From Butch: need to initialize the state at the beginning to idle
        state_ = State.Idle ;
    }

    public boolean shooterValue() {
        return shooter_value_ ;
    }

    //
    // From Butch: this method is part of the simulation support.  It returns the ball count to the
    //             simulator to compare against expected ball counts
    //
    @Override
    public SettingsValue getProperty(String name) {
        SettingsValue v = null ;

        if (name.equals("ball-count")) {
            v = new SettingsValue(getBallCount()) ;
        }
        else {
            v = super.getProperty(name) ;
        }

        return v ;
    }

    //
    // From Butch: may be needed by other subsystems or the OI to know how many balls are being held
    //
    public int getBallCount() {
        return ball_count_ ;
    }

    //
    // From Butch: needed by the OI to know if the conveyor is holding two balls
    //
    public boolean isFull() {
        return ball_count_ == 2 ;
    }

    private String stateToString(Boolean b1, Boolean b2, Boolean b3) {
        return b1.toString()  + ", " + b2.toString() + ", " + b3.toString() ;
    }

    @Override
    public void computeMyState() throws Exception {
        super.computeMyState();

        String prev = stateToString(intake_value_, middle_value_, shooter_value_) ;

        intake_value_ = !intake_sensor_.get();
        shooter_value_ = !shooter_sensor_.get();
        middle_value_ = !middle_sensor_.get();

        String current = stateToString(intake_value_, middle_value_, shooter_value_) ;

        //
        // From Butch: added these five lines to see when the sensors change state
        //
        if (!prev.equals(current)) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getLoggerID()) ;
            logger.add("sensors changed: ").add(prev.toString()).add(" --> ").add(current.toString()).endMessage();
        }

        putDashboard("intake-bb", DisplayType.Verbose, intake_value_);
        putDashboard("shooter-bb", DisplayType.Verbose, shooter_value_);
        putDashboard("middle-bb", DisplayType.Verbose, middle_value_);
    }

    public void collect() {
        if (state_ == State.Idle) {
            state_ = State.WaitForIntake; 
        }
    }

    public void shoot() {
        if (state_ == State.Idle) {
            state_ = State.StartShoot ;
        }
    }

    public void shoot(double dur) {
        if (state_ == State.Idle) {
            if (dur != Double.NaN) {
                shoot_timer_.setDuration(dur);
            }
            state_ = State.StartShoot ;
        }
    }

    public void stop() {
        isStop = true;
    }

    public void eject() {
        setPower(eject_power_);
        eject_timer_.start();
        state_ = State.Eject; 
    }

    public boolean isIdle() {
        return state_ == State.Idle; 
    }

    public void run() throws Exception {
        super.run() ;

        State prev = state_ ;

        switch (state_) {
            case Idle:
                IdleProc();
                break;

            case WaitForIntake:
                WaitForIntakeProc();
                break;

            case WaitForMiddle:
                WaitForMiddleProc();
                break;

            case WaitForShooter:
                WaitForShooterProc();
                break;

            case WaitForIntake2:
                WaitForIntake2Proc();
                break;

            case Eject:
               EjectProc(); 
               break;

            case StartShoot:
                StartShootProc() ;
                break ;

            case Shooting:
                ShootingProc() ;
                break ;

            case PreloadBallWaitForMiddleLow:
                PreloadBallWaitForMiddleLow() ;
                break ;

            case PreloadBallWaitForMiddleHigh:
                PreloadBallWaitForMiddleHigh() ;
                break ;                

        }

        //
        // From Butch: I added these five lines so the log file will tell you when you change
        //             states in the conveyor
        //
        if (prev != state_) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getLoggerID()) ;
            logger.add("state changed: ").add(prev.toString()).add(" --> ").add(state_.toString()).endMessage();
        }
    }

    private void WaitForIntakeProc() {
        //
        // In this method, we have started a collect operation but currently have no balls.
        // We are waiting for a ball to break the intake sensor at which point we will start
        // the conveyor motor to move the ball into the conveyor. 
        //
        if(isStop) {
            setPower(off_power_);
            state_ = State.Idle; 
        }
        if (intake_value_ == true) {
            setPower(collect_power_);
            state_ = State.WaitForMiddle;
        }
    }

    private void WaitForMiddleProc() {
        MessageLogger logger = getRobot().getMessageLogger() ;
        //
        // After the intake sensor is broken and the motor is turned on, we check to see
        // if there is a second ball at the intake sensor. If there is, we keep the motor running 
        // and wait for the shooter sensor to be broken. If the middle sensor is broken
        // and there isn't another ball behind it, we turn off the motor to park the ball in the middle. 
        //
        if (intake_value_ == true && middle_value_ == true) {
            logger.startMessage(MessageType.Debug) ;
            logger.add("Saw two at once") ;
            logger.endMessage();
            setPower(collect_power_);
            ball_count_ = 2;
            state_ = State.WaitForShooter;
        } else if (middle_value_ == true) {
            logger.startMessage(MessageType.Debug) ;
            logger.add("Saw only one") ;
            logger.endMessage();
            setPower(off_power_);
            ball_count_ = 1;
            state_ = State.WaitForIntake2;
        }
    }

    private void WaitForShooterProc() {
        //
        // In this case, we will always have two balls in the conveyor. Here, we are waiting for the
        // shooter sensor to be broken so that we can park the balls there. After we stop the motors, we set
        // the state to WaitForIntake. (should it be set to idle??)
        //
        if (shooter_value_ == true) {
            setPower(off_power_);
            ball_count_ = 2;
            state_ = State.Idle;
        }
    }

    private void WaitForIntake2Proc() {
        //
        // In this case, we have one ball stationed at the middle sensor. We are waiting for a second ball
        // to break the intake sensor so that we can turn on the motor power and move the balls towards the shooter.
        //
        if(isStop) {
            setPower(off_power_);
            state_ = State.Idle;
        }
        if (intake_value_ == true && middle_value_ == true) {
            setPower(collect_power_);
            ball_count_ = 1; 
            state_ = State.WaitForShooter;
        }
    }
     
    private void EjectProc() {
        if(eject_timer_.isExpired()) {
            ball_count_ = 0; 
            setPower(off_power_); 
            state_ = State.Idle; 
        }
    }

    private void IdleProc() {
        if(isStop){
            isStop = false; 
        }
        //if 2+ ball count, we're already in idle state, so we don't switch state!
    }

    private void StartShootProc() {
        if (isStop) {
            setPower(off_power_) ;
            state_ = State.Idle ;
        }
        else {
            setPower(shoot_power_) ;
            state_ = State.Shooting ;
            shoot_timer_.start() ;
        }
    }

    private void ShootingProc() {
        if (isStop || shoot_timer_.isExpired()) {
            setPower(off_power_) ;
            state_ = State.Idle ;
            ball_count_ = 0 ;
        }
    }
    
    private void PreloadBallWaitForMiddleLow() {
        if (!middle_value_) {
            setPower(preload_power_);
            state_ = State.PreloadBallWaitForMiddleHigh ;
        }
    }

    private void PreloadBallWaitForMiddleHigh() {
        if (middle_value_) {
            setPower(off_power_);
            state_ = State.Idle ;
        }
    }

    public void setPreloadedBall() {
        MessageLogger logger = getRobot().getMessageLogger() ;

        if (state_ == State.Idle) {
            ball_count_ = 1;
            if (middle_value_) {
                setPower(-preload_power_);
                state_ = State.PreloadBallWaitForMiddleLow ;
            }
            else {
                setPower(preload_power_);
                state_ = State.PreloadBallWaitForMiddleHigh ;
            }
        }
        else {
            logger.startMessage(MessageType.Error).add("Conveyor.setPreloadBall called when the conveyor was not idle").endMessage();
        }
    }
}
