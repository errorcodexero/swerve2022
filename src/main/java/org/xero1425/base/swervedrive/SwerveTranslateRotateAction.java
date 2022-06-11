package org.xero1425.base.swervedrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveTranslateRotateAction extends SwerveDriveAction {
    private ChassisSpeeds speeds_ ;
    
    public SwerveTranslateRotateAction(SwerveDriveSubsystem sub, ChassisSpeeds sp) {
        super(sub) ;

        speeds_ = sp ;
    }

    public SwerveTranslateRotateAction(SwerveDriveSubsystem sub) {
        super(sub) ;
        speeds_ = new ChassisSpeeds() ;
    }

    public void update(ChassisSpeeds speeds) {
        speeds_ = speeds ;
    }

    @Override
    public void start() {
        getSubsystem().drive(speeds_) ;
    }

    @Override
    public void run() {
        getSubsystem().drive(speeds_) ;
    }

    @Override
    public String toString(int indent) {
        return "SwerveTranslateRotateAction x=" + speeds_.vxMetersPerSecond + ", y=" + speeds_.vyMetersPerSecond + ", ang=" + speeds_.omegaRadiansPerSecond ;
    }
}
