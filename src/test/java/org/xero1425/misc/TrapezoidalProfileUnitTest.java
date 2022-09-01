package org.xero1425.misc ;

import org.junit.*;

public class TrapezoidalProfileUnitTest
{
    @Before
    public void init() {
    }

    @Test
    public void testPositiveV() {
        TrapezoidalProfile profile = new TrapezoidalProfile(2, -4, 100) ;
        profile.update(96.0, 0.0, 0.0) ;

        Assert.assertEquals(profile.getTimeAccel(), 8.0, 1e-6) ;
        Assert.assertEquals(profile.getTimeCruise(), 0.0, 1e-6) ;
        Assert.assertEquals(profile.getTimeDecel(), 4.0, 1e-6) ;
        Assert.assertEquals(profile.getActualMaxVelocity(), 16.0, 1e-6) ;
        Assert.assertEquals(profile.getDistance(4.0), 16.0, 1e-6) ;
        Assert.assertEquals(profile.getVelocity(4.0), 8.0, 1e-6) ;
        Assert.assertEquals(profile.getDistance(10.0), 88.0, 1e-6) ;
        Assert.assertEquals(profile.getVelocity(10.0), 8.0, 1e-6) ;
    }

    @Test
    public void testNegativeV() {
        TrapezoidalProfile profile = new TrapezoidalProfile(2, -4, 100) ;
        profile.update(-96.0, 0.0, 0.0) ;

        Assert.assertEquals(profile.getTimeAccel(), 8.0, 1e-6) ;
        Assert.assertEquals(profile.getTimeCruise(), 0.0, 1e-6) ;
        Assert.assertEquals(profile.getTimeDecel(), 4.0, 1e-6) ;
        Assert.assertEquals(profile.getActualMaxVelocity(), -16.0, 1e-6) ;
        Assert.assertEquals(profile.getDistance(4.0), -16.0, 1e-6) ;
        Assert.assertEquals(profile.getVelocity(4.0), -8.0, 1e-6) ;
        Assert.assertEquals(profile.getDistance(10.0), -88.0, 1e-6) ;
        Assert.assertEquals(profile.getVelocity(10.0), -8.0, 1e-6) ;
    }  
    
    @Test
    public void testPositiveVNonZeroStart() {
        TrapezoidalProfile profile = new TrapezoidalProfile(2, -6, 100) ;
        profile.update(309, 12.0, 22.0) ;

        Assert.assertEquals(profile.getTimeAccel(), 11.0, 1e-6) ;
        Assert.assertEquals(profile.getTimeCruise(), 0.0, 1e-6) ;
        Assert.assertEquals(profile.getTimeDecel(), 2.0, 1e-6) ;
        Assert.assertEquals(profile.getActualMaxVelocity(), 34.0, 1e-6) ;
        Assert.assertEquals(profile.getDistance(9.0), 189.0, 1e-6) ;
        Assert.assertEquals(profile.getVelocity(9.0), 30.0, 1e-6) ;
        Assert.assertEquals(profile.getDistance(12.0), 284.0, 1e-6) ;
        Assert.assertEquals(profile.getVelocity(12.0), 28.0, 1e-6) ;
    }

    @Test
    public void testNegativeVNonZeroEnd() {
        TrapezoidalProfile profile = new TrapezoidalProfile(2, -6, 100) ;
        profile.update(-309.0, -12.0, -22.0) ;

        Assert.assertEquals(profile.getTimeAccel(), 11.0, 1e-6) ;
        Assert.assertEquals(profile.getTimeCruise(), 0.0, 1e-6) ;
        Assert.assertEquals(profile.getTimeDecel(), 2.0, 1e-6) ;
        Assert.assertEquals(profile.getActualMaxVelocity(), -34.0, 1e-6) ;
        Assert.assertEquals(profile.getDistance(9.0), -189.0, 1e-6) ;
        Assert.assertEquals(profile.getVelocity(9.0), -30.0, 1e-6) ;
        Assert.assertEquals(profile.getDistance(12.0), -284.0, 1e-6) ;
        Assert.assertEquals(profile.getVelocity(12.0), -28.0, 1e-6) ;
    }      

    @Test
    public void testDecelOnly() {
        TrapezoidalProfile profile = new TrapezoidalProfile(2, -4, 40) ;
        profile.update(200, 40, 0) ;

        Assert.assertEquals(profile.getTimeAccel(), 0.0, 1e-6) ;
        Assert.assertEquals(profile.getTimeCruise(), 0.0, 1e-6) ;
        Assert.assertEquals(profile.getTimeDecel(), 10.0, 1e-6) ;
        Assert.assertEquals(profile.getActualMaxVelocity(), 40.0, 1e-6) ;
        Assert.assertEquals(profile.getDistance(4.0), 128.0, 1e-6) ;
        Assert.assertEquals(profile.getVelocity(4.0), 24.0, 1e-6) ;
        Assert.assertEquals(profile.getDistance(7.0), 182.0, 1e-6) ;
        Assert.assertEquals(profile.getVelocity(7.0), 12.0, 1e-6) ;
    }     

    @Test
    public void testCruiseDecelOnly() {
        TrapezoidalProfile profile = new TrapezoidalProfile(2, -8, 40) ;
        profile.update(500, 40, 0) ;

        Assert.assertEquals(profile.getTimeAccel(), 0.0, 1e-6) ;
        Assert.assertEquals(profile.getTimeCruise(), 10.0, 1e-6) ;
        Assert.assertEquals(profile.getTimeDecel(), 5.0, 1e-6) ;
        Assert.assertEquals(profile.getActualMaxVelocity(), 40.0, 1e-6) ;
        Assert.assertEquals(profile.getDistance(5.0), 200.0, 1e-6) ;
        Assert.assertEquals(profile.getVelocity(5.0), 40.0, 1e-6) ;
        Assert.assertEquals(profile.getDistance(13.0), 484.0, 1e-6) ;
        Assert.assertEquals(profile.getVelocity(13.0), 16.0, 1e-6) ;
    }       
    
    @Test
    public void testTrapezoid() {
        TrapezoidalProfile profile = new TrapezoidalProfile(2, -4, 20) ;
        profile.update(550, 0, 0) ;

        Assert.assertEquals(profile.getTimeAccel(), 10.0, 1e-6) ;
        Assert.assertEquals(profile.getTimeCruise(), 20.0, 1e-6) ;
        Assert.assertEquals(profile.getTimeDecel(), 5.0, 1e-6) ;
        Assert.assertEquals(profile.getActualMaxVelocity(), 20.0, 1e-6) ;
        Assert.assertEquals(profile.getDistance(5.0), 25.0, 1e-6) ;
        Assert.assertEquals(profile.getVelocity(5.0), 10.0, 1e-6) ;
        Assert.assertEquals(profile.getDistance(20.0), 300.0, 1e-6) ;
        Assert.assertEquals(profile.getVelocity(20.0), 20.0, 1e-6) ;
        Assert.assertEquals(profile.getDistance(33.0), 542.0, 1e-6) ;
        Assert.assertEquals(profile.getVelocity(33.0), 8.0, 1e-6) ;        
    }

    @Test
    public void testComplexTrapezoid() {
        TrapezoidalProfile profile = new TrapezoidalProfile(9, -13, 95) ;
        profile.update(2564, 5, 69) ;

        Assert.assertEquals(profile.getTimeAccel(), 10.0, 1e-6) ;
        Assert.assertEquals(profile.getTimeCruise(), 20.0, 1e-6) ;
        Assert.assertEquals(profile.getTimeDecel(), 2.0, 1e-6) ;
        Assert.assertEquals(profile.getActualMaxVelocity(), 95.0, 1e-6) ;
    }
}
