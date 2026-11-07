package org.firstinspires.ftc.teamcode.purepursuit;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.purepursuit.paths.BioBuzzRoutes;
import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;

public class MechanismBindingsTest {
    @Test
    public void everyMarkerStepGetsAFreshCommand() {
        MechanismBindings<Object> bindings = new MechanismBindings<>().register("shoot", Object::new);
        assertNotSame(bindings.create("shoot"), bindings.create("shoot"));
    }

    @Test
    public void findsMarkersWithNothingRegistered() {
        Route route = Route.builder("r", Route.Alliance.NONE, new Pose2d(0, -48, 0))
                .waitSeconds(1)
                .marker("shoot", 2)
                .parallelMarker("intaek", 2)
                .marker("shoot", 2)
                .build();
        MechanismBindings<Object> bindings = new MechanismBindings<>().register("shoot", Object::new);
        assertEquals(Collections.singletonList("intaek"), bindings.unbound(route));
    }

    @Test
    public void theMechAutosBindEveryMarker() {
        MechanismBindings<Object> bindings = new MechanismBindings<>()
                .register(BioBuzzRoutes.SHOOT, Object::new)
                .register(BioBuzzRoutes.INTAKE, Object::new);
        for (Route route : Arrays.asList(BioBuzzRoutes.BLUE_MECH_SHORT, BioBuzzRoutes.BLUE_MECH_CORNER,
                BioBuzzRoutes.BLUE_MECH_FLOWER, BioBuzzRoutes.RED_MECH_SHORT, BioBuzzRoutes.RED_MECH_CORNER,
                BioBuzzRoutes.RED_MECH_FLOWER)) {
            assertTrue(route.name + " " + bindings.unbound(route), bindings.unbound(route).isEmpty());
        }
    }

    @Test(expected = IllegalArgumentException.class)
    public void aMarkerCanOnlyBeRegisteredOnce() {
        new MechanismBindings<>().register("shoot", Object::new).register("shoot", Object::new);
    }
}
