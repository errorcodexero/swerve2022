package org.xero1425.misc ;

import org.junit.*;

public class PIDACtrlUnitTest
{
    @Before
    public void init() {

    }

    @Test
    public void testVOnly() {
        final PIDACtrl p = new PIDACtrl(0.01, 0.0, 0.0, 0.0, false);
        Assert.assertEquals(p.getOutput(100, 10, 2.0, 2.0, 0.02), 0.1, 1e-6) ;
    }
}