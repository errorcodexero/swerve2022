package org.xero1425.simulator.models;

public class WheelSubModel {
    private SimMotorController motor_ ;
    private double circum_ ;
    private double meters_per_second_per_power_ ;
    private double last_position_ ;
    private double position_ ;
    private double speed_ ;

    public WheelSubModel(SimMotorController motor, double diameter, double meters_per_second_per_power) {
        motor_ = motor ;
        circum_ = diameter * Math.PI ;
        meters_per_second_per_power_ = meters_per_second_per_power ;

        last_position_ = 0.0 ;
        position_ = 0.0 ;
        speed_ = 0.0 ;
    }

    public void run(double dt) {
        double power = motor_.getPower() ;
        position_ = last_position_ + meters_per_second_per_power_ * power * dt ;
        speed_ = (position_ - last_position_) / dt ;
        last_position_ = position_ ;

        if (motor_.usesTicks()) {
            double revs = (position_ / circum_) ;
            double ticks = revs * motor_.ticksPerRev() ;
            motor_.setEncoder(ticks);
        }
    }

    public double getPosition() {
        return position_ ;
    }

    public double getSpeed() {
        return speed_ ;
    }
}
