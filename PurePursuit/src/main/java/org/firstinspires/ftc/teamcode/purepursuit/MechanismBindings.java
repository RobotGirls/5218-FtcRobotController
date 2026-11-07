package org.firstinspires.ftc.teamcode.purepursuit;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

/**
 * Connects a route's marker names to the mechanism code that runs at them, so mechanism classes
 * (Intake, Launcher, ...) stay in TeamCode while the routes stay simulatable here.
 *
 * <pre>
 * new MechanismBindings&lt;Action&gt;()
 *         .register(BioBuzzRoutes.SHOOT, launcher::launcherForward)
 *         .register(BioBuzzRoutes.INTAKE, intake::intakeIn);
 * </pre>
 *
 * Each marker step gets a fresh command from its factory, so a route can shoot twice. The
 * simulator only uses a marker's estimated seconds; on the robot the command decides when the
 * marker is done.
 *
 * @param <T> the robot's command type, Road Runner's Action on the robot
 */
public final class MechanismBindings<T> {
    private final Map<String, Supplier<? extends T>> factories = new LinkedHashMap<>();

    /** Runs factory's command at every marker with this name. */
    public MechanismBindings<T> register(String markerName, Supplier<? extends T> factory) {
        if (markerName == null || markerName.trim().isEmpty() || factory == null) {
            throw new IllegalArgumentException("a marker name and a mechanism are both needed");
        }
        if (factories.containsKey(markerName)) {
            throw new IllegalArgumentException("marker '" + markerName + "' is already registered");
        }
        factories.put(markerName, factory);
        return this;
    }

    public boolean supports(String markerName) {
        return factories.containsKey(markerName);
    }

    public Set<String> names() {
        return Collections.unmodifiableSet(factories.keySet());
    }

    /** A fresh command for one marker step. */
    public T create(String markerName) {
        Supplier<? extends T> factory = factories.get(markerName);
        if (factory == null) {
            throw new IllegalArgumentException("no mechanism registered for marker '" + markerName + "'");
        }
        T command = factory.get();
        if (command == null) {
            throw new IllegalStateException("the mechanism for marker '" + markerName + "' returned null");
        }
        return command;
    }

    /**
     * Marker names in the route with nothing registered (waits don't count). On the robot those
     * markers just wait, so a name here is usually a typo.
     */
    public List<String> unbound(Route route) {
        List<String> missing = new ArrayList<>();
        for (Route.Step step : route.steps) {
            if (step instanceof Route.Marker) {
                String name = ((Route.Marker) step).name;
                if (!name.equals(Route.WAIT) && !supports(name) && !missing.contains(name)) {
                    missing.add(name);
                }
            }
        }
        return missing;
    }
}
