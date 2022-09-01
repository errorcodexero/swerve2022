package frc.robot.subsystems.conveyor;

import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.wpilibj.DigitalInput;

//
// Code Review Notes (from Butch)
// 
// - We need to keep up with a ball count.  When shooting and other operations, like
//   lighting LEDs on the OI we need to know how many balls we are holding.  Put a ball_count_
//   variable in the class and provide a method to return it.  Then think about where in the
//   state machine you know you have a given number of balls.
//

public class ConveyorSubsystem extends MotorSubsystem {

    private enum State {
        Idle,
        WaitForIntake,
        WaitForMiddle,
        WaitForShooter,
        WaitForIntake2,
        Eject
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

    private int ball_count_;

    private XeroTimer eject_timer_;

    private boolean isStop; 


    public ConveyorSubsystem(Subsystem parent) throws BadParameterTypeException, MissingParameterException {
        super(parent, "conveyor");

        collect_power_ = getSettingsValue("power:collect").getDouble();
        off_power_ = getSettingsValue("power:off").getDouble();
        eject_power_ = getSettingsValue("power:eject").getDouble();

        //
        // Code Review Notes (from Butch)
        // 1. You need to create the sensors here.  I put in the code for one of these to show an example, but
        //    you need this for all three sensor.  I also put a value for the intake sensor I/O in the swerve2022.json
        //    file under the intake subsystem.
        //
        int channel = getSettingsValue("sensors:intake").getInteger() ;
        intake_sensor_ = new DigitalInput(channel) ;

        int channel1 = getSettingsValue("sensors:middle").getInteger() ;
        middle_sensor_ = new DigitalInput(channel1) ;

        int channel2 = getSettingsValue("sensors:shooter").getInteger() ;
        shooter_sensor_ = new DigitalInput(channel2) ;

        eject_timer_ = new XeroTimer(getRobot(), "Eject", getSettingsValue("eject_duration").getDouble());

        isStop = false; 

    }

    @Override
    public void computeMyState() throws Exception {
        super.computeMyState();

        intake_value_ = !intake_sensor_.get();
        shooter_value_ = !shooter_sensor_.get();
        middle_value_ = !middle_sensor_.get();
    }

    public void collect() {
        if (state_ == State.Idle) {
            state_ = State.WaitForIntake; 
        }
        
    }

    public void shoot() {

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

    //
    // Code Review Notes (from Butch)
    // Other functions you are going to need here to control state:
    //   public void collect() - starts a collect operation (moves you from Idle state to WaitForIntake state)
    //   public void shoot() - will need this eventually, but don't worry about it for now, other than putting the method in
    //   public void stop() - stop the current operation as soon as is possible
    //   public void eject() - starts an eject operation (moves you from Idle state to a new Eject state you need to add)
    //

    public void run() {
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
        //
        // After the intake sensor is broken and the motor is turned on, we check to see
        // if there is a second ball at the intake sensor. If there is, we keep the motor running 
        // and wait for the shooter sensor to be broken. If the middle sensor is broken
        // and there isn't another ball behind it, we turn off the motor to park the ball in the middle. 
        //
        if (intake_value_ == true && middle_value_ == true) {
            setPower(collect_power_);
            ball_count_ = 2;
            state_ = State.WaitForShooter;
        } else if (middle_value_ == true) {
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
}