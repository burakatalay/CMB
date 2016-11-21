package movement;

import core.Coord;
import core.Settings;

import java.util.*;

/**
 * Random Waypoint Movement with a prohibited region where nodes may not move
 * into. The polygon is defined by a *closed* (same point as first and
 * last) path, represented as a list of {@code Coord}s.
 *
 * @author teemuk
 */
public class ProhibitedPolygonRwp
extends MovementModel {

  //==========================================================================//
  // Settings
  //==========================================================================//
  /** {@code true} to confine nodes inside the polygon */
  public static final String INVERT_SETTING = "rwpInvert";
  public static final boolean INVERT_DEFAULT = false;
  //==========================================================================//


  //==========================================================================//
  // Instance vars
  //==========================================================================//
  final List<Coord> attractionPoints = Arrays.asList(
          new Coord( 1650.0, 400.0 ),
          new Coord( 2000.0, 200.0 ),
          new Coord( 1650.0, 900.0 ),
          new Coord( 1650.0, 900.0 ),
          new Coord( 1330.0, 100.0 ),
          new Coord( 930.0, 100 ),
          new Coord( 530.0, 100 ),
          new Coord( 130.0, 100 ),
          new Coord( 1330.0, 900.0 ),
          new Coord( 930.0, 900 ),
          new Coord( 530.0, 900 ),
          new Coord( 530.0, 900 ),
          new Coord( 530.0, 700 )
  );

  final List<Coord> pathPoints = Arrays.asList(
          new Coord( 1650.0, 400.0 ),
          new Coord( 1330.0, 400.0 ),
          new Coord( 930.0, 400.0 ),
          new Coord( 530.0, 400.0 ),
          new Coord( 930.0, 600.0 ),
          new Coord( 1330.0, 600.0 ),
          new Coord( 1650.0, 600.0 )
  );

  final List <Coord> polygon = Arrays.asList(

          new Coord( 0, 0 ),
          new Coord( 0, 557 ),
          new Coord( 408, 561 ),
          new Coord( 408, 632 ),
          new Coord( 113, 629 ),
          new Coord( 107, 868 ),
          new Coord( 649, 864 ),
          new Coord( 649, 668 ),
          new Coord( 826, 668 ),
          new Coord( 826, 1039 ),
          new Coord( 1003, 1039 ),
          new Coord( 1008, 657 ),
          new Coord( 1234, 664 ),
          new Coord( 1239, 1036 ),
          new Coord( 1432, 1036 ),
          new Coord( 1427, 661 ),
          new Coord( 1598, 657 ),
          new Coord( 1604, 1047 ),
          new Coord( 1786, 1047 ),
          new Coord( 1781, 657 ),
          new Coord( 1920, 654 ),
          new Coord( 1920, 1057 ),
          new Coord( 2178, 1054 ),
          new Coord( 2167, 539 ),
          new Coord( 2065, 543 ),
          new Coord( 2065, 454 ),
          new Coord( 2349, 454 ),
          new Coord( 2355, 179 ),
          new Coord( 1888, 182 ),
          new Coord( 1899, 343 ),
          new Coord( 1432, 347 ),
          new Coord( 1421, 7 ),
          new Coord( 1250, 4 ),
          new Coord( 1244, 343 ),
          new Coord( 1008, 347 ),
          new Coord( 1003, 4 ),
          new Coord( 821, 0 ),
          new Coord( 831, 354 ),
          new Coord( 654, 354 ),
          new Coord( 654, 3 ),
          new Coord( 418, 0 ),
          new Coord( 424, 339 ),
          new Coord( 214, 343 ),
          new Coord( 214, 4 ),
          new Coord( 0, 0 )
  );

  private Coord lastWaypoint;
  /** Inverted, i.e., only allow nodes to move inside the polygon. */
  private final boolean invert;
  //==========================================================================//



  //==========================================================================//
  // Implementation
  //==========================================================================//
  @Override
  public Path getPath() {
    // Creates a new path from the previous waypoint to a new one.
    final Path p;
    p = new Path( super.generateSpeed() );
    p.addWaypoint( this.lastWaypoint.clone() );

    // Add only one point. An arbitrary number of Coords could be added to
    // the path here and the simulator will follow the full path before
    // asking for -the next one.

    LinkedList<Coord> path = new LinkedList<>();

    int randomInt;
    randomInt = rng.nextInt(attractionPoints.size());
    Coord finalPoint = attractionPoints.get(randomInt);

    Coord nextPoint = null;

    if(this.lastWaypoint.getX() != finalPoint.getX()) {
      do {
        randomInt = rng.nextInt(pathPoints.size());
        nextPoint = pathPoints.get(randomInt);
      }
      while (nextPoint.getX() < finalPoint.getX() || (nextPoint.equals(this.lastWaypoint))
              || nextPoint.getX() > this.lastWaypoint.getX() || pathIntersects(this.polygon, this.lastWaypoint, nextPoint));
    }

    if(this.lastWaypoint.getX() == finalPoint.getX() ) {
      nextPoint = finalPoint;
      p.addWaypoint(nextPoint);
      this.lastWaypoint = nextPoint;
    } else{
      p.addWaypoint(nextPoint);
      this.lastWaypoint = nextPoint;
    }

return p;


  }

    public LinkedList getPossiblePath(Coord currentPoint, Coord finalPoint, LinkedList<Coord> path){

        if( (path.getLast().getX() == finalPoint.getX() && path.getLast().getY() == finalPoint.getY()) || this.lastWaypoint.equals(finalPoint) ){
            path.add(finalPoint);
            return path;
        }else{
            for (Coord coord: this.attractionPoints){
                if(currentPoint.getX() == coord.getX() || currentPoint.getY() == coord.getY()){
                    path.add(coord);
                }
            }
        }

        return null;
    }

  @Override
  public Coord getInitialLocation() {
    /*this.lastWaypoint = new Coord(113420, 482000);
    return this.lastWaypoint; */
  /* do {
      this.lastWaypoint = this.randomCoord();
    } while ( ( this.invert ) ?
        isOutside( polygon, this.lastWaypoint ) :
        isInside( this.polygon, this.lastWaypoint ) );
    return this.lastWaypoint; */
    int randomInt = rng.nextInt(attractionPoints.size());
    this.lastWaypoint =   attractionPoints.get(randomInt);
    return this.lastWaypoint;
  }

  @Override
  public MovementModel replicate() {
    return new ProhibitedPolygonRwp( this );
  }

  private Coord randomCoord() {
    return new Coord(
        rng.nextDouble() * super.getMaxX(),
        rng.nextDouble() * super.getMaxY());
          // rng.nextDouble() * 113300,
           // rng.nextDouble() * 483000);
  }
  //==========================================================================//


  //==========================================================================//
  // API
  //==========================================================================//
  public ProhibitedPolygonRwp( final Settings settings ) {
    super( settings );
    // Read the invert setting
    this.invert = settings.getBoolean( INVERT_SETTING, INVERT_DEFAULT );
  }

  public ProhibitedPolygonRwp( final ProhibitedPolygonRwp other ) {
    // Copy constructor will be used when settings up nodes. Only one
    // prototype node instance in a group is created using the Settings
    // passing constructor, the rest are replicated from the prototype.
    super( other );
    // Remember to copy any state defined in this class.
    this.invert = other.invert;
  }
  //==========================================================================//


  //==========================================================================//
  // Private - geometry
  //==========================================================================//
  private static boolean pathIntersects(
      final List <Coord> polygon,
      final Coord start,
      final Coord end ) {
    final int count = countIntersectedEdges( polygon, start, end );
    return ( count > 0 );
  }

  private static boolean isInside(
      final List <Coord> polygon,
      final Coord point ) {
    final int count = countIntersectedEdges( polygon, point,
        new Coord( -10,0 ) );
    return ( ( count % 2 ) != 0 );
  }

  private static boolean isOutside( final List <Coord> polygon, final Coord point ) {
    return !isInside( polygon, point );
  }

  private static int countIntersectedEdges(final List <Coord> polygon, final Coord start, final Coord end ) {

    int count = 0;
    for ( int i = 0; i < polygon.size() - 1; i++ ) {
      final Coord polyP1 = polygon.get( i );
      final Coord polyP2 = polygon.get( i + 1 );

      final Coord intersection = intersection( start, end, polyP1, polyP2 ); // intersection of two lines
      if ( intersection == null ) continue;

      if ( isOnSegment( polyP1, polyP2, intersection ) && isOnSegment( start, end, intersection ) ) {
        count++;
      }
    }
    return count;

  }

  private static boolean isOnSegment(final Coord L0, final Coord L1, final Coord point ) {

      final double crossProduct = ( point.getY() - L0.getY() ) * ( L1.getX() - L0.getX() )
                                - ( point.getX() - L0.getX() ) * ( L1.getY() - L0.getY() );

    if ( Math.abs( crossProduct ) > 0.0000001 ) return false;

    final double dotProduct
        = ( point.getX() - L0.getX() ) * ( L1.getX() - L0.getX() )
        + ( point.getY() - L0.getY() ) * ( L1.getY() - L0.getY() );
    if ( dotProduct < 0 ) return false;

    final double squaredLength
        = ( L1.getX() - L0.getX() ) * ( L1.getX() - L0.getX() )
        + (L1.getY() - L0.getY() ) * (L1.getY() - L0.getY() );
    if ( dotProduct > squaredLength ) return false;

    return true;
  }

  private static Coord intersection(final Coord L0_p0, final Coord L0_p1, final Coord L1_p0, final Coord L1_p1 ) {

    final double[] p0 = getParams( L0_p0, L0_p1 );
    final double[] p1 = getParams( L1_p0, L1_p1 );
    final double D = p0[ 1 ] * p1[ 0 ] - p0[ 0 ] * p1[ 1 ];

      if ( D == 0.0 ) return null;

    final double x = ( p0[ 2 ] * p1[ 1 ] - p0[ 1 ] * p1[ 2 ] ) / D;
    final double y = ( p0[ 2 ] * p1[ 0 ] - p0[ 0 ] * p1[ 2 ] ) / D;

    return new Coord( x, y );
  }

  private static double[] getParams( final Coord c0,  final Coord c1 ) {

    final double A = c0.getY() - c1.getY();
    final double B = c0.getX() - c1.getX();
    final double C = c0.getX() * c1.getY() - c0.getY() * c1.getX();

    return new double[] { A, B, C };
  }
  //==========================================================================//
}
