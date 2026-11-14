package org.firstinspires.ftc.teamcode.purepursuit.paths;

import org.firstinspires.ftc.teamcode.purepursuit.Route;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/** Every route, in the order the FieldViewer and the path tester OpMode list them. */
public final class Routes {
    private Routes() {}

    public static final List<Route> ALL = Collections.unmodifiableList(Arrays.asList(
            BioBuzzRoutes.BLUE_FLOWER_PARK,
            BioBuzzRoutes.BLUE_GARDEN_PARK,
            BioBuzzRoutes.BLUE_CORNER_PARK,
            BioBuzzRoutes.BLUE_GARDEN_SHOOTER,
            BioBuzzRoutes.BLUE_STARTERBOT_SHOOT,
            BioBuzzRoutes.RED_STARTERBOT_SHOOT,
            BioBuzzRoutes.RED_FLOWER_PARK,
            BioBuzzRoutes.RED_GARDEN_PARK,
            BioBuzzRoutes.RED_CORNER_PARK,
            BioBuzzRoutes.BLUE_MECH_SHORT,
            BioBuzzRoutes.BLUE_MECH_CORNER,
            BioBuzzRoutes.BLUE_MECH_FLOWER,
            BioBuzzRoutes.RED_MECH_SHORT,
            BioBuzzRoutes.RED_MECH_CORNER,
            BioBuzzRoutes.RED_MECH_FLOWER,
            TestRoutes.STRAIGHT_24,
            TestRoutes.STRAIGHT,
            TestRoutes.OUT_AND_BACK,
            TestRoutes.FIELD_LOOP,
            TestRoutes.U_TURN,
            TestRoutes.S_CURVE,
            TestRoutes.TURN_IN_PLACE,
            TestRoutes.STRAFE_SQUARE,
            TestRoutes.TUNING_COURSE,
            TestRoutes.MY_ROUTE,
            TestRoutes.MY_ROUTE2,
            TestRoutes.MY_ROUTE_SLOW,
            BioBuzzRoutes.BLUE_GARDEN_PARK_COPY
    ));

    /** Looks a route up by name, or returns null. */
    public static Route byName(String name) {
        for (Route route : ALL) {
            if (route.name.equals(name)) {
                return route;
            }
        }
        return null;
    }
}
