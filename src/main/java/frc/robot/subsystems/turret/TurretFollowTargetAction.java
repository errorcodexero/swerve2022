package frc.robot.subsystems.turret;

import org.xero1425.base.subsystems.motorsubsystem.MotorAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.PIDCtrl;

import frc.robot.subsystems.targettracker.TargetTrackerSubsystem;

//
// This action is assigned to the turret in order to have the turret follow any target
// seen by the limelight.  This action never completes and will have to be canceled by assigning
// another action to the turret.
//
public class TurretFollowTargetAction extends MotorAction {
    
    // The turret must be less than the threshold from the desired angle (as determined by the target tracker)
    // in order to be ready to fire
    private double threshold_ ;

    // The PID controller that actually calculates the power required to have the turret track the target
    private PIDCtrl pid_ ;

    // The turret subsystem
    private TurretSubsystem sub_ ;

    // The target tacker subsystem, used to find the desired turret angle
    private TargetTrackerSubsystem tracker_ ;

    private double desired_ ;
    private double error_ ;

    private boolean locked_ ;

    public TurretFollowTargetAction(TurretSubsystem sub, TargetTrackerSubsystem tracker)
            throws BadParameterTypeException, MissingParameterException {
        super(sub);

        sub_ = sub ;
        tracker_ = tracker ;
        locked_ = false ;
        threshold_ = sub.getSettingsValue("fire_threshold").getDouble() ;
    }

    public void lock(boolean l) {
        locked_ = l ;
    }

    public double getDesired() {
        return desired_ ;
    }

    public double getError() {
        return error_ ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        //
        // This PID controller does the work of following the target
        //
        pid_ = new PIDCtrl(getSubsystem().getRobot().getSettingsSupplier(), "subsystems:turret:follow", false);

        //
        // Enable the target tracker, this lights up the LED lights and starts the flow of data
        //
        tracker_.enable(true);
    }

    @Override
    public void run() {
        double out = 0.0 ;
        MessageLogger logger = sub_.getRobot().getMessageLogger() ;

        //
        // Ask the target tracker what angle the turret should be at to 
        // point at the target.
        //

        if (tracker_.hasTarget()) {
            desired_ = tracker_.getDesiredTurretAngle() ;

            if (desired_ > 60.0)
                desired_ = 60.0 ;
            else if (desired_ < -55.0)
                desired_ = -55.0 ;
            //
            // Update the turret motor power based on the current position of the turret and
            // the desired positon of the turret.
            //
            if (!locked_) {
                out = pid_.getOutput(desired_, sub_.getPosition(), sub_.getRobot().getDeltaTime()) ;
            }
            sub_.setPower(out) ;

            //
            // Determine if the turret is close enough to the desired position to enable 
            // firing of the balls
            //
            double error_ = Math.abs(desired_ - sub_.getPosition()) ;
            boolean ready = error_ < threshold_ ;
            sub_.setReadyToFire(ready) ;

            //
            // Print debug messages for this action
            //
            logger.startMessage(MessageType.Debug, sub_.getLoggerID()) ;
            logger.add("FollowTargetAction:") ;
            logger.add(" desired", desired_) ;
            logger.add(" position", sub_.getPosition()) ;
            logger.add(" error", error_) ;
            logger.add(" output", out) ;
            logger.add(" ready", ready) ;
            logger.endMessage();            
        }
        else {
            logger.startMessage(MessageType.Debug, sub_.getLoggerID()) ;
            logger.add("FollowTargetAction: no target detected") ;
            logger.endMessage();
            sub_.setReadyToFire(false);
            out = pid_.getOutput(0.0, sub_.getPosition(), sub_.getRobot().getDeltaTime()) ;
            sub_.setPower(out) ;
        }
    }

    @Override
    public void cancel() {
        super.cancel() ;

        //
        // If we are being canceled, the turret will not be ready to fire
        //
        sub_.setReadyToFire(false) ;

        //
        // Disable the target tracker, this turns off the LED lights
        //
        tracker_.enable(false);

        //
        // Set the turret power to zero
        //
        sub_.setPower(0.0) ;
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "FollowTargetAction" ;
    }

}