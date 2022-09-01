package frc.robot.subsystems;

import org.xero1425.base.motors.MotorController;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.wpilibj.DigitalInput;

//
// Code Review Notes (from Butch)
// - You need to add the "hw" section of the swerve2022.json file for the
//   conveyor motor.  I would do this immediately as it then allows you to run the
//   simulator as you develop code.  Just copy the motor section from some other subsystem
//   (like the intake), but don't copy the encoder part.
//
// - We need to keep up with a ball count.  When shooting and other operations, like
//   lighting LEDs on the OI we need to know how many balls we are holding.  Put a ball_count_
//   variable in the class and provide a method to return it.  Then think about where in the
//   state machine you know you have a given number of balls.
//

public class ConveyorSubsystem extends MotorSubsystem {

    //
    // Code Review Notes (from Butch)
    // 1. I moved these next lines from later in the file. We generally use a
    // pattern of member variables, followed by
    // public methods, followed by private methods. This is a real common pattern in
    // Java in general.
    //
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


    //
    // Code Review Notes (from Butch)
    // You don't need to create this MotorController.  Since you are derived from the MotorSubsystem,
    // the base class will create the motor.  You can control its motor power with setPower() like you 
    // do already below.
    // private MotorController conveyor_motor_;     <<<----- not going to be needed
    //

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

    }

    //
    // Code Review Notes (from Butch)
    // 1. The name of this method is computeMyState(), I changed the upper case 'C'
    // to lower case
    // 2. If you add the @Override before the method, Java will ensure you are
    // overridding a method that exists in the base class
    // 3. I added the call super.computeMyState() which calls the base class to let
    // it do any work it needs to do
    // 4. I added throws Exception since the base class may throw exceptions
    //
    @Override
    public void computeMyState() throws Exception {
        super.computeMyState();

        intake_value_ = !intake_sensor_.get();
        shooter_value_ = !shooter_sensor_.get();
        middle_value_ = !middle_sensor_.get();
    }

    public void collect() {
        
        
    }

    public void shoot() {

    }

    public void stop() {

    }

    public void eject() {

    }

    //
    // Code Review Notes (from Butch)
    // Other functions you are going to need here to control state:
    //   public void collect() - starts a collect operation (moves you from Idle state to WaitForIntake state)
    //   public void shoot() - will need this eventually, but don't worry about it for now, other than putting the method in
    //   public void stop() - stop the current operation as soon as is possible
    //   public void eject() - starts an eject operation (moves you from Idle state to a new Eject state you need to add)
    //

    //
    // Code Review Notes (from Butch)
    // 1. Fixed the indenting, should be consistant
    //
    public void run() {
        switch (state_) {
            case Idle:
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
        }
    }

    private void WaitForIntakeProc() {
        //
        // Code Review Notes (from Butch)
        //   I would add comments at the start of each state method detailing what is
        //   going on.  For instance ...
        //
        // In this method, we have started a collect operation but currently have no balls.
        // We are waiting for a ball to break the intake sensor at which point we will start
        // the conveyor motor to move the ball into the conveyor. 
        //
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
            state_ = State.WaitForShooter;
        } else if (middle_value_ == true) {
            setPower(off_power_);
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
            state_ = State.WaitForIntake;
        }
    }

    private void WaitForIntake2Proc() {
        //
        // Code Review Notes (from Butch)
        //   I am not sure about the logic here.  The WaitForIntake2 state is when you are waiting on 
        //   the second ball to break the intake sensor.  You always already have one ball when you are
        //   in this state.
        //
        // (I changed the logic to something that I think makes sense)
        // In this case, we have one ball stationed at the middle sensor. We are waiting for a second ball
        // to break the intake sensor so that we can turn on the motor power and move the balls towards the shooter.
        //
        if (intake_value_ == true && middle_value_ == true) {
            setPower(collect_power_);
            state_ = State.WaitForShooter;
        }
    }
}