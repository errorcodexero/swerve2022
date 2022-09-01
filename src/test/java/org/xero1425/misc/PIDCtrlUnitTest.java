package org.xero1425.misc ;

import org.junit.*;

public class PIDCtrlUnitTest
{
    @Before
    public void init() {

    }

    @Test
    public void testPOnly() {
        final PIDCtrl p = new PIDCtrl(0.01, 0.0, 0.0, 0.0, -1.0, 1.0, 10.0, false);
        Assert.assertEquals(p.getOutput(100, 98, 0.02), 0.02, 1e-6) ;
    }
}