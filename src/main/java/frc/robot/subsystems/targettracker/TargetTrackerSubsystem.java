package frc.robot.subsystems.targettracker;

import org.xero1425.base.limelight.LimeLightSubsystem;
import org.xero1425.base.limelight.LimeLightSubsystem.LedMode;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.utils.TargetTracker;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.turret.TurretSubsystem;


//
// The purpose of the tracker class is to generate two things.  It generates
// the desired angle for the TURRET pid controller and it generates the distance
// from the robot camera to the target.  This goes to false if the lime light loses
// the target for an extended number of robot loops.
//
public class TargetTrackerSubsystem extends Subsystem {
    private boolean enabled_ ;
    private double desired_turret_angle_ ;
    private LimeLightSubsystem ll_ ;
    private TurretSubsystem turret_ ;
    private double distance_ ;
    private int lost_count_ ;
    private int max_lost_count_ ;
    private boolean has_vision_target_ ;
    private TrackMethod track_method_ ;
    private TargetTracker field_target_tracker_ ;

    // Lock the target to a fixed distance/angle. Don't track based on vision or field.
    private boolean lock_enabled_ ;
    private double  lock_angle_ ;
    private double  lock_distance_ ;

    private double camera_offset_angle_ ;

    public static final String SubsystemName = "targettracker" ;

    /// \brief What method to use for target tracking
    public enum TrackMethod
    {
        VisionOrFieldPosition,   ///< (vf) Use limelight if vision seen, else use drivebase field position tracker
        VisionOnly,              ///< (v)  Limelight only
        FieldPositionOnly        ///< (f)  Drivebase field position tracker only
    } ;

    public TargetTrackerSubsystem(Subsystem parent, LimeLightSubsystem ll, TurretSubsystem turret)
            throws BadParameterTypeException, MissingParameterException, Exception {

        super(parent, SubsystemName);

        ll_ = ll ;
        turret_ = turret ;

        desired_turret_angle_ = 0.0 ;
        lost_count_ = 0 ;
        has_vision_target_ = false ;
        lock_enabled_ = false ;

        Translation2d target_coordinates = new Translation2d(27*12, 13.5*12) ;
        field_target_tracker_ = new TargetTracker(target_coordinates) ;
        
        //
        // Camera offset angle is determined empirically and deals with any angular offset in the
        // mounting or manufacturing of the camera.
        //
        camera_offset_angle_ = getSettingsValue("camera_offset_angle").getDouble() ;

        //
        // This is the number of robot loops that the lime light can lose the target before we assume
        // the target is lost for firing.  When a ball is fired, it temporarily blocks the view of the target
        // for the limelight.  Therefore if we lose the target for short amounts of time, we just maintain the
        // values we last calculated until the target is seen again.  Only after this number of robot loops
        // without a target do we actually consider the target lost and stop the firing operation.
        //
        max_lost_count_ = getSettingsValue("lost_count").getInteger() ;

        // Assign track method from settings file
        String method = getSettingsValue("track_method").getString();
        if (method.equals("vf")) {
            track_method_ = TrackMethod.VisionOrFieldPosition;
        } else if (method.equals("v")) {
            track_method_ = TrackMethod.VisionOnly;
        } else if (method.equals("f")) {
            track_method_ = TrackMethod.FieldPositionOnly;
        } else {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("paramater ").addQuoted("track_method").add(" has invalid value. Setting to vf.") ;
            logger.endMessage() ;
            track_method_ = TrackMethod.VisionOrFieldPosition;
        }

        //
        // Turn off the LEDs unless we are actually wanting to track a target
        //
        enable(false) ;

        if (isSettingDefined("forced-target") && isSettingDefined("forced-distance") && isSettingDefined("forced-angle")) {
            if (getSettingsValue("forced-target").getBoolean()) {
                double dist = getSettingsValue("forced-distance").getDouble();
                double angle = getSettingsValue("forced-angle").getDouble();
                lockTarget(angle, dist) ;

                MessageLogger logger = getRobot().getMessageLogger() ;
                logger.startMessage(MessageType.Info) ;
                logger.add("The target tracker has been forced on for testing") ;
                logger.endMessage() ;
            }
        }
    }

    public LimeLightSubsystem getLimelight() {
        return ll_ ;
    }

    public void enable(boolean b) {
        enabled_ = b ;

        if (ll_ != null) {
            if (b)
            {
                ll_.setLedMode(LedMode.ForceOn);
            }
            else
            {
                ll_.setLedMode(LedMode.ForceOff);
            }
        }
    }

    public boolean hasVisionTarget() {
        return has_vision_target_ ;
    }

    public boolean hasTarget() {
        return lock_enabled_ || has_vision_target_ || (track_method_ != TrackMethod.VisionOnly) ;
    }

    public double getDesiredTurretAngle() {
        return desired_turret_angle_ ;
    }

    public double getDistance() {
        return distance_ ;
    }

    public void lockTarget(double angle, double distance) {
        lock_enabled_ = true ;
        lock_angle_ = angle ;
        lock_distance_ = distance ;
    }

    public void unlockTarget() {
        lock_enabled_ = false ;
    }

    @Override
    public void computeMyState() {
        MessageLogger logger = getRobot().getMessageLogger() ;

        if (enabled_)
        {
            if (lock_enabled_)
            {
                desired_turret_angle_ = lock_angle_ ;
                distance_ = lock_distance_ ;    
            }
            else
            {
                if (ll_.isTargetDetected() && (track_method_ != TrackMethod.FieldPositionOnly))
                {
                    distance_ = ll_.getDistance() ;
                
                    double yaw = ll_.getYaw() - camera_offset_angle_ ;
                    desired_turret_angle_ = -yaw + turret_.getPosition() ;
                    logger.startMessage(MessageType.Debug, getLoggerID());
                    logger.add("yaw", yaw).add("distance", distance_) ;
                    logger.add(" ll", ll_.getYaw()).add(" offset", camera_offset_angle_) ;
                    logger.add(" tpos", turret_.getPosition()).add(" desired", desired_turret_angle_);
                    logger.endMessage();

                    has_vision_target_ = true ;
                    lost_count_ = 0 ;
                }
                else if (track_method_ != TrackMethod.VisionOnly) 
                {
                    // Use field position
                    Pose2d robot_pose = getRobot().getRobotSubsystem().getDB().getPose() ;
                    double target_angle = field_target_tracker_.getRelativeTargetAngle(robot_pose) ;
                    double safe_target_angle = target_angle ;
                    
                    desired_turret_angle_ = safe_target_angle ;
                    distance_ = field_target_tracker_.getRelativeTargetDistance(robot_pose) ;
                }

                // Using track method that allows vision, but target is not detected
                if (!ll_.isTargetDetected() && (track_method_ != TrackMethod.FieldPositionOnly))
                {
                    lost_count_++ ;
                    
                    if (lost_count_ > max_lost_count_)
                        has_vision_target_ = false ;

                    logger.startMessage(MessageType.Debug, getLoggerID());
                    logger.add("targettracker: lost target ").add(" lost count", lost_count_) ;
                    logger.add(" has_target", has_vision_target_).endMessage();
                }
            }

        }
        else
        {
            //
            // If the target tracker is disabled, we set the desired angle to zero, which is
            // straight ahead.
            //
            distance_ = 0.0 ;
            desired_turret_angle_ = 0.0 ;
        }
    }
}