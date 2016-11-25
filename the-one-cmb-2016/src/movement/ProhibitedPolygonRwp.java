package movement;

import core.Coord;
import core.Settings;
import core.SimClock;

import java.lang.reflect.Array;
import java.util.*;

/**
 * Random Waypoint Movement with a prohibited region where nodes may not move
 * into. The polygon is defined by a *closed* (same point as first and
 * last) path, represented as a list of {@code Coord}s.
 *
 * @author teemuk
 */
public class ProhibitedPolygonRwp extends MovementModel {

    //==========================================================================//
    // Settings
    //==========================================================================//
    /**
     * {@code true} to confine nodes inside the polygon
     */
    public static final String INVERT_SETTING = "rwpInvert";
    public static final boolean INVERT_DEFAULT = false;

    public int randomInt;
    public Random random = new Random();
    //==========================================================================//


    //==========================================================================//
    // Instance vars
    //==========================================================================//
    final List<Coord> attractionPoints = Arrays.asList(
            new Coord(2250.0, 400.0),
            new Coord(1350.0, 100.0),
            new Coord(900.0, 100.0),
            new Coord(500.0, 100.0),
            // new Coord( 100.0, 100.0 ),
            new Coord(530.0, 800.0),
            new Coord(900.0, 1000.0),
            new Coord(1350.0, 1000.0),
            new Coord(1700.0, 1000.0),
            new Coord(2000.0, 1000.0),
            new Coord(530.0, 700)
    );


    Map<Coord, ArrayList<Double>> attractionPointsMap = new HashMap<Coord, ArrayList<Double>>() {{
        put( new Coord(1350.0, 100.0), new ArrayList<Double>() {{ add(10.0); }});
        put( new Coord(900.0, 100.0), new ArrayList<Double>() {{ add(20.0); }});
        put( new Coord(500.0, 100.0), new ArrayList<Double>() {{ add(20.0); }});
        put( new Coord(530.0, 800.0), new ArrayList<Double>() {{ add(40.0); }});
        put( new Coord(900.0, 1000.0), new ArrayList<Double>() {{ add(40.0); }});
        put( new Coord(1350.0, 1000.0), new ArrayList<Double>() {{ add(40.0); }});
        put( new Coord(1700.0, 1000.0), new ArrayList<Double>() {{ add(60.0); }});
        put( new Coord(2000.0, 1000.0), new ArrayList<Double>() {{ add(100.0); }});
        put( new Coord(530.0, 700), new ArrayList<Double>() {{ add(400.0); }});
    }};


    final List<Coord> pathPoints = Arrays.asList(
            new Coord(1650.0, 400.0),
            new Coord(1330.0, 400.0),
            new Coord(930.0, 400.0),
            new Coord(530.0, 400.0),
            new Coord(930.0, 600.0),
            new Coord(1330.0, 600.0),
            new Coord(1650.0, 600.0)
    );

    final List<Coord> insidePolygon = Arrays.asList(
            new Coord(2000.0, 400.0),
            new Coord(2000.0, 650.0),
            new Coord(430.0, 400.0),
            new Coord(430.0, 650.0),
            new Coord(2000.0, 400.0)
    );

    final List<Coord> test = Arrays.asList(
            new Coord(1350.0, 650.0),
            new Coord(530.0, 600.0),
            new Coord(750.0, 450.0),
            new Coord(1000.0, 650.0),
            new Coord(1000.0, 400.0)
    );

    final List<Coord> polygon = Arrays.asList(
            new Coord(0, 0),
            new Coord(0, 557),
            new Coord(408, 561),
            new Coord(408, 632),
            new Coord(113, 629),
            new Coord(107, 868),
            new Coord(649, 864),
            new Coord(649, 668),
            new Coord(826, 668),
            new Coord(826, 1039),
            new Coord(1003, 1039),
            new Coord(1008, 657),
            new Coord(1234, 664),
            new Coord(1239, 1036),
            new Coord(1432, 1036),
            new Coord(1427, 661),
            new Coord(1598, 657),
            new Coord(1604, 1047),
            new Coord(1786, 1047),
            new Coord(1781, 657),
            new Coord(1920, 654),
            new Coord(1920, 1057),
            new Coord(2178, 1054),
            new Coord(2167, 539),
            new Coord(2065, 543),
            new Coord(2065, 454),
            new Coord(2349, 454),
            new Coord(2355, 179),
            new Coord(1888, 182),
            new Coord(1899, 343),
            new Coord(1432, 347),
            new Coord(1421, 7),
            new Coord(1250, 4),
            new Coord(1244, 343),
            new Coord(1008, 347),
            new Coord(1003, 4),
            new Coord(821, 0),
            new Coord(831, 354),
            new Coord(654, 354),
            new Coord(654, 3),
            new Coord(418, 0),
            new Coord(424, 339),
            new Coord(214, 343),
            new Coord(214, 4),
            new Coord(0, 0)
    );

    private Coord lastWaypoint;
    private Coord finalPoint = null;
    private int timeSlot = 3500;

    /**
     * Inverted, i.e., only allow nodes to move inside the polygon.
     */
    private final boolean invert;
    //==========================================================================//


    //==========================================================================//
    // Implementation
    //==========================================================================//
    @Override
    public Path getPath() {
        // Creates a new path from the previous waypoint to a new one.
        final Path p;
        p = new Path(super.generateSpeed());
        p.addWaypoint(this.lastWaypoint.clone());


        // Add only one point. An arbitrary number of Coords could be added to
        // the path here and the simulator will follow the full path before
        // asking for -the next one.
        Coord nextPoint;


        if (this.finalPoint == null) {
            getFinalCoordination();
        }

        Coord currentPoint = this.lastWaypoint;

        if (SimClock.getIntTime() % timeSlot == 0) {
            getFinalCoordination();
        } else {
            if (!this.finalPoint.equals(currentPoint)) {
                do {
                    nextPoint = getNextLocation();
                    if (!pathIntersects(this.polygon, currentPoint, nextPoint)) {
                        if (isOutside(this.insidePolygon, currentPoint)) {
                            p.addWaypoint(nextPoint);
                            this.lastWaypoint = nextPoint;
                            return p;
                        } else {
                            if (calculateDistance(nextPoint, finalPoint) < calculateDistance(currentPoint, finalPoint)) {
                                p.addWaypoint(nextPoint);
                                this.lastWaypoint = nextPoint;
                                return p;
                            }
                        }
                    }
                } while (calculateDistance(nextPoint, finalPoint) < calculateDistance(currentPoint, finalPoint));


                if (calculateDistance(currentPoint, finalPoint) < 500) {
                    if (!pathIntersects(this.polygon, currentPoint, finalPoint)) {
                        p.addWaypoint(finalPoint);
                        this.lastWaypoint = finalPoint;
                        return p;

                    } else {
                        do {
                            nextPoint = getNextLocation();
                            if (!(calculateDistance(nextPoint, finalPoint) > calculateDistance(currentPoint, finalPoint))) {
                                currentPoint = this.lastWaypoint;
                                if (!pathIntersects(this.polygon, currentPoint, nextPoint)) {

                                    p.addWaypoint(nextPoint);
                                    this.lastWaypoint = nextPoint;
                                    return p;
                                }
                            }


                        }
                        while ((!pathIntersects(this.polygon, currentPoint, nextPoint)) && (calculateDistance(nextPoint, finalPoint) < calculateDistance(currentPoint, finalPoint)));
                    }
                }
            } else {
                return p;
            }

        }


        return p;
    }

    private void getFinalCoordination() {
        double randomDouble = random.nextDouble();

        for (Map.Entry<Coord, ArrayList<Double>> entry : this.attractionPointsMap.entrySet()) {
            double lowerBound = entry.getValue().get(1);
            double upperBound = entry.getValue().get(2);
            if (lowerBound <= randomDouble && randomDouble <= upperBound) {
                this.finalPoint = entry.getKey();
                break;
            }
        }
    }

    public double calculateDistance(Coord c1, Coord c2) {
        return Math.sqrt(((Math.pow((c1.getX() - c2.getX()), 2)) + Math.pow((c1.getY() - c2.getY()), 2)));
    }

    @Override
    public Coord getInitialLocation() {
        randomInt = random.nextInt(this.attractionPointsMap.size());

        Set set = this.attractionPointsMap.entrySet();
        Iterator iterator = set.iterator();

        int i = 0;

        while(iterator.hasNext() && i<=randomInt){
            Map.Entry mapEntry = (Map.Entry) iterator.next();
            if(i == randomInt){
                this.lastWaypoint = (Coord) mapEntry.getKey();
                break;
            }
            i++;
        }

        //this.lastWaypoint = new Coord(1350.0, 1000.0);
        return this.lastWaypoint;

    }

    public Coord getNextLocation() {
        Coord nextPoint;
        do {
            nextPoint = this.randomCoord();
        } while (!isInside(this.insidePolygon, nextPoint));
        return nextPoint;
    }

    @Override
    public MovementModel replicate() {

        return new ProhibitedPolygonRwp(this);
    }

    private void calculateProbabilities() {

        double totalCapacity = 0.0;

        for (Map.Entry<Coord, ArrayList<Double>> entry : this.attractionPointsMap.entrySet()) {
            totalCapacity += entry.getValue().get(0);
        }

        double totalProbability = 0.0;

        for (Map.Entry<Coord, ArrayList<Double>> entry : this.attractionPointsMap.entrySet()) {
            double roomCapacity = entry.getValue().get(0);
            double probability = roomCapacity / totalCapacity;

            this.attractionPointsMap.get(entry.getKey()).add(totalProbability);

            totalProbability += probability;

            this.attractionPointsMap.get(entry.getKey()).add(totalProbability);
        }

    }

    private Coord randomCoord() {
        return new Coord(
                random.nextDouble() * super.getMaxX(),
                random.nextDouble() * super.getMaxY());
    }
    //==========================================================================//


    //==========================================================================//
    // API
    //==========================================================================//
    public ProhibitedPolygonRwp(final Settings settings) {
        super(settings);
        // Read the invert setting
        this.invert = settings.getBoolean(INVERT_SETTING, INVERT_DEFAULT);
        calculateProbabilities();
    }

    public ProhibitedPolygonRwp(final ProhibitedPolygonRwp other) {
        // Copy constructor will be used when settings up nodes. Only one
        // prototype node instance in a group is created using the Settings
        // passing constructor, the rest are replicated from the prototype.
        super(other);
        // Remember to copy any state defined in this class.
        this.invert = other.invert;
        calculateProbabilities();
    }
    //==========================================================================//


    //==========================================================================//
    // Private - geometry
    //==========================================================================//
    private static boolean pathIntersects(
            final List<Coord> polygon,
            final Coord start,
            final Coord end) {
        final int count = countIntersectedEdges(polygon, start, end);
        return (count > 0);
    }

    private static boolean isInside(
            final List<Coord> polygon,
            final Coord point) {
        final int count = countIntersectedEdges(polygon, point,
                new Coord(-10, 0));
        return ((count % 2) != 0);
    }

    private static boolean isOutside(final List<Coord> polygon, final Coord point) {
        return !isInside(polygon, point);
    }

    private static int countIntersectedEdges(final List<Coord> polygon, final Coord start, final Coord end) {

        int count = 0;
        for (int i = 0; i < polygon.size() - 1; i++) {
            final Coord polyP1 = polygon.get(i);
            final Coord polyP2 = polygon.get(i + 1);

            final Coord intersection = intersection(start, end, polyP1, polyP2); // intersection of two lines
            if (intersection == null) continue;

            if (isOnSegment(polyP1, polyP2, intersection) && isOnSegment(start, end, intersection)) {
                count++;
            }
        }
        return count;

    }

    private static boolean isOnSegment(final Coord L0, final Coord L1, final Coord point) {

        final double crossProduct = (point.getY() - L0.getY()) * (L1.getX() - L0.getX())
                - (point.getX() - L0.getX()) * (L1.getY() - L0.getY());

        if (Math.abs(crossProduct) > 0.0000001) return false;

        final double dotProduct
                = (point.getX() - L0.getX()) * (L1.getX() - L0.getX())
                + (point.getY() - L0.getY()) * (L1.getY() - L0.getY());
        if (dotProduct < 0) return false;

        final double squaredLength
                = (L1.getX() - L0.getX()) * (L1.getX() - L0.getX())
                + (L1.getY() - L0.getY()) * (L1.getY() - L0.getY());
        if (dotProduct > squaredLength) return false;

        return true;
    }

    private static Coord intersection(final Coord L0_p0, final Coord L0_p1, final Coord L1_p0, final Coord L1_p1) {

        final double[] p0 = getParams(L0_p0, L0_p1);
        final double[] p1 = getParams(L1_p0, L1_p1);
        final double D = p0[1] * p1[0] - p0[0] * p1[1];

        if (D == 0.0) return null;

        final double x = (p0[2] * p1[1] - p0[1] * p1[2]) / D;
        final double y = (p0[2] * p1[0] - p0[0] * p1[2]) / D;

        return new Coord(x, y);
    }

    private static double[] getParams(final Coord c0, final Coord c1) {

        final double A = c0.getY() - c1.getY();
        final double B = c0.getX() - c1.getX();
        final double C = c0.getX() * c1.getY() - c0.getY() * c1.getX();

        return new double[]{A, B, C};
    }
    //==========================================================================//
}
