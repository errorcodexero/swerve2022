package org.xero1425.base.oi;

import org.xero1425.base.DriveBaseSubsystem;
import org.xero1425.base.LoopType;
import org.xero1425.base.actions.SequenceAction;
import org.xero1425.base.swervedrive.SwerveDirectionRotateAction;
import org.xero1425.base.swervedrive.SwerveDriveSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.wpilibj.DriverStation;

public class SwerveDriveGamepad extends Gamepad {
    private SwerveDriveSubsystem db_;
    private double angle_maximum_;
    private double pos_maximum_;
    private double deadband_pos_x_ ;
    private double deadband_pos_y_ ;
    private double deadband_rotate_ ;
    private double power_ ;
    private SwerveDirectionRotateAction action_;

    public SwerveDriveGamepad(OISubsystem oi, int index, DriveBaseSubsystem drive_) throws Exception {
        super(oi, "swerve_gamepad", index);

        if (DriverStation.getStickPOVCount(getIndex()) == 0) {
            throw new Exception("invalid gamepad for TankDriveGamepad");
        }

        if (DriverStation.getStickAxisCount(getIndex()) <= AxisNumber.RIGHTX.value) {
            throw new Exception("invalid gamepad for TankDriveGamepad");
        }

        db_ = (SwerveDriveSubsystem)drive_;
        if (db_ == null) {
            throw new Exception("invalid drivebase for SwerveDriveGamepad - expected swerve drive");            
        }
    }
    
    @Override
    public void init(LoopType ltype) {
    }

    @Override
    public void createStaticActions() throws BadParameterTypeException, MissingParameterException {
        deadband_pos_x_ = getSubsystem().getSettingsValue("swerve_gamepad:position:deadband:x").getDouble() ;
        deadband_pos_y_ = getSubsystem().getSettingsValue("swerve_gamepad:position:deadband:y").getDouble() ;
        deadband_rotate_ = getSubsystem().getSettingsValue("swerve_gamepad:angle:deadband").getDouble() ;
        action_ = new SwerveDirectionRotateAction(db_, 0.0, 0.0, 0.0) ;
    }

    @Override
    public void computeState() {
        super.computeState();
    }

    @Override
    public void generateActions(SequenceAction seq) {
        if (db_ == null || !isEnabled())
            return ;

        // For X axis, left is -1, right is +1
        // For Y axis, forward is -1, back is +1

        double ly = DriverStation.getStickAxis(getIndex(), AxisNumber.LEFTY.value) ;
        double lx = DriverStation.getStickAxis(getIndex(), AxisNumber.LEFTX.value) ;
        double rx = DriverStation.getStickAxis(getIndex(), AxisNumber.RIGHTX.value) ;

        double lyscaled = mapJoyStick(ly, pos_maximum_, deadband_pos_y_, power_) ;
        double lxscaled = mapJoyStick(lx, pos_maximum_, deadband_pos_x_, power_) ;
        double rxscaled = mapJoyStick(rx, angle_maximum_, deadband_rotate_, power_) ;

        //
        // The rotational velocity is given by rxscaled
        // The position velocity is given by the vector (lyscaled, lxscaled)
        //
        // Note, the x and y are swapped because of the orientation of the gamepad versus the orientation of the
        // field.  The drive team is at the end of the field looking down the X axis.  So, when the Y axis on the
        // gamepad is pushed forward (negative value from the gamepad), the driver expects the robot to move along
        // the positive X axis of the field.  
        //
        action_.updateTargets(-lyscaled, -lxscaled, -rxscaled) ;
        if (db_.getAction() != action_)
            db_.setAction(action_) ;
    }    

    private double mapJoyStick(double v, double maxv, double db, double power) {
        if (Math.abs(v) < db)
            return 0.0 ;

        return Math.signum(v) * Math.pow(Math.abs(v), power) * maxv ;
    }
}