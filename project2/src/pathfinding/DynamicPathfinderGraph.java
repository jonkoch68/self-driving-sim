package pathfinding;

import java.util.ArrayList;
import java.util.List;

import brain.SmartCarBrain;
import core.Car;
import core.Goal;
import core.Road;
import core.World;
import processing.core.PApplet;
import processing.core.PVector;

public class DynamicPathfinderGraph extends PathfinderGraph {

	public DynamicPathfinderGraph ( Car car, World world, Goal goal ) {
		super(car,world,goal);
	}

	private void addLaneChange ( RoadGraphNode node, int newlane,
	                             List<RoadGraphNode> targets ) {
		Road road = node.getRoad();
		float time = node.getTime();

		// compute the target location in the new lane
		// node's position is front bumper
		PVector target = road.getLaneChangeTarget(node.getPosition(),newlane,
		                                          car_.LANE_CHANGE_ANGLE);

		// check that there is room in the new lane for the car - one car length
		// back from target, with spacing on either end - at the time we expect to
		// get there
		float t = PVector.dist(node.getPosition(),target) / car_.getMaxSpeed();
		// make sure there aren't other cars in the way
		List<Car> interval =
		    world_.getCarsInInterval(road,target,-car_.getLength() - World.SPACING,
		                             World.SPACING,time + t);
		if ( interval.size() > 1 ) {
			return;
		}
		if ( interval.size() == 1 && !interval.contains(car_) ) {
			return;
		}

		RoadGraphNode change = new RoadGraphNode(road,target,time + t);
		targets.add(change);
		world_.debugPoint(World.DEBUG_GRAPHGEN,change.getPosition(),
		                  world_.getApplet().color(0,255,255),8);
		System.out.println("lane change node (" + newlane + "): " + change);

	}

	/**
	 * Get the time for car's front bumper to reach World.SPACING behind ahead's
	 * rear bumper based on car traveling at max speed until braking (if brake is
	 * true) or just at max speed and ahead traveling at current speed.
	 * 
	 * @param node
	 *          car's current position and time
	 * @param ahead
	 *          next car ahead in the lane
	 * @return time for car's front bumper to reach World.SPACING behind ahead's
	 *         rear bumper based on car travel
	 */
	private float getTimeToCatch ( RoadGraphNode node, Car ahead,
	                               boolean brake ) {
		float brakedist = (brake ? car_.getBrakingDist(ahead.getSpeed()) : 0);
		float braketime = (brake ? car_.getBrakingTime(ahead.getSpeed()) : 0);
		float curdist =
		    PVector.dist(node.getPosition(),ahead.getRearBumper(node.getTime()))
		        - World.SPACING;

		return (braketime * car_.getMaxSpeed() + curdist - brakedist)
		    / (car_.getMaxSpeed() - ahead.getSpeed());
	}

	private float getTimeToPass ( RoadGraphNode node, Car pass, Car ahead,
	                              boolean brake ) {
		float curdist =
		    PVector.dist(node.getPosition(),pass.getFrontBumper(node.getTime()))
		        + car_.getLength() + World.SPACING;

		return curdist / (car_.getMaxSpeed() - pass.getSpeed());
	}

	private void addOpportunity ( RoadGraphNode node, int newlane, Car ahead,
	                              List<RoadGraphNode> targets ) {
		// in the current lane, immediately after passing each car in the adjacent
		// lane, until reaching the next car in the current lane
		// safe opportunity - car travels at max speed until braking just before
		// next car in lane
		// unsafe opportunity - car travels at max speed

		Road road = node.getRoad();
		int lane = road.getLane(node.getPosition());
		float time = node.getTime();

		// compute point and time where braking starts (only a brake point if there
		// is a car ahead and it is moving slower)
		float tb = Float.MAX_VALUE; // time when braking starts
		PVector pb = road.getEnd(lane); // point where braking starts
		if ( ahead != null && ahead.getSpeed() < car_.getMaxSpeed() ) {
			float brakedist = car_.getBrakingDist(ahead.getSpeed());
			float braketime = car_.getBrakingTime(ahead.getSpeed());
			tb = (braketime * car_.getMaxSpeed()
			    + PVector.dist(node.getPosition(),ahead.getRearBumper(node.getTime()))
			    - World.SPACING - brakedist)
			    / (car_.getMaxSpeed() - ahead.getSpeed());
			pb = road.getAlong(node.getPosition(),tb * car_.getMaxSpeed());
			world_.debugPoint(World.DEBUG_GRAPHGEN,pb,car_.getColor(),6);
		}

		for ( Car adjacent : world_.getCars() ) {
			// only consider cars in the desired adjacent lane
			if ( adjacent.getLane() != newlane ) {
				continue;
			}

			// if other car is going same speed or faster than we can go, can't pass
			// (but could slot in behind once it has gone by - not accounted for)
			if ( adjacent.getSpeed() >= car_.getMaxSpeed() ) {
				continue;
			}

			// if other car is behind where we would change lanes to, it has already
			// been passed
			PVector target = road.getLaneChangeTarget(node.getPosition(),newlane,
			                                          Car.LANE_CHANGE_ANGLE);
			PVector threshold =
			    road.getAlong(target,-car_.getLength() - World.SPACING);
			world_.debugCircle(World.DEBUG_GRAPHGEN,threshold,
			                   world_.getApplet().color(255,255,255),6);
			if ( road.compareTo(adjacent.getFrontBumper(time),threshold) <= 0 ) {
				continue;
			}

			// compute passing point and time to get there at max speed
			float t = PVector.dist(target,
			                       road.getAlong(adjacent.getFrontBumper(time),
			                                     car_.getLength() + World.SPACING))
			    / (car_.getMaxSpeed() - adjacent.getSpeed());
			PVector p = road.getAlong(node.getPosition(),t * car_.getMaxSpeed());

			// only consider passing point if we're not already there
			if ( t < .000001 ) {
				continue;
			}

			// only consider passing point if not ahead of the next car ahead (and not
			// past the end of the road)
			if ( road.compareTo(p,road.getEnd(lane)) > 0
			    || (ahead != null && road.compareTo(p,road
			        .getAlong(ahead.getRearBumper(time + t),-World.SPACING)) > 0) ) {
				continue;
			}

			// make sure there aren't other cars in the way of where we want to merge
			// in
			List<Car> interval = world_
			    .getCarsInInterval(road,target,-car_.getLength() - World.SPACING,
			                       World.SPACING,time + t);
			if ( interval.size() > 1 ) {
				continue;
			}
			if ( interval.size() == 1 && !interval.contains(car_) ) {
				continue;
			}

			if ( ahead != null ) {
				world_.debugCircle(World.DEBUG_GRAPHGEN,
				                   road.getAlong(ahead.getCenter(time + t),0),
				                   ahead.getLength() / 2,ahead.getColor());
			}
			world_.debugCircle(World.DEBUG_GRAPHGEN,
			                   road.getAlong(adjacent.getCenter(time + t),0),
			                   adjacent.getLength() / 2,adjacent.getColor());

			// unsafe opportunity if passing point is after braking point
			if ( road.compareTo(p,pb) > 0 ) {
				RoadGraphNode unsafe = new RoadGraphNode(road,p,time + t);
				targets.add(unsafe);
				world_.debugPoint(World.DEBUG_GRAPHGEN,unsafe.getPosition(),
				                  world_.getApplet().color(255,0,0),8);
				world_.debugPoint(World.DEBUG_GRAPHGEN,
				                  road.getLaneChangeTarget(unsafe.getPosition(),newlane,
				                                           Car.LANE_CHANGE_ANGLE),
				                  world_.getApplet().color(255,0,0),6);
			}

			if ( road.compareTo(p,pb) <= 0 ) {
				// safe opportunity - passing point is before braking point - travel at
				// full speed
				RoadGraphNode safe = new RoadGraphNode(road,p,time + t);
				targets.add(safe);
				world_.debugPoint(World.DEBUG_GRAPHGEN,safe.getPosition(),
				                  world_.getApplet().color(0,255,0),8);
				world_.debugPoint(World.DEBUG_GRAPHGEN,
				                  road.getLaneChangeTarget(safe.getPosition(),newlane,
				                                           Car.LANE_CHANGE_ANGLE),
				                  world_.getApplet().color(0,255,0),6);
			} else {
				// safe opportunity - braking starts before desired passing point, so
				// need to take slowing into account
				float a = car_.getMaxAccel() / 2;
				float b = car_.getMaxSpeed() - adjacent.getSpeed();
				float c = (t - tb) * (car_.getMaxSpeed() - adjacent.getSpeed());
				if ( b * b - 4 * a * c < 0 ) {
					continue;
				}
				// time to passing point while braking
				float tp = -b + (float) Math.sqrt(b * b - 4 * a * c) / (2 * a);
				PVector ps = road.getAlong(node.getPosition(),t * car_.getMaxSpeed()
				    + car_.getMaxSpeed() * tp + car_.getMaxAccel() / 2 * tp * tp);

				// safe opportunity - if that point is not ahead of the next car ahead
				// in this lane (or past the end of the road), create the opportunity
				// node
				if ( road.compareTo(ps,road.getAlong(ahead.getRearBumper(time + t + tp),
				                                     -World.SPACING)) <= 0 ) {
					RoadGraphNode safe = new RoadGraphNode(road,p,time + t + tp);
					targets.add(safe);
					world_.debugPoint(World.DEBUG_GRAPHGEN,safe.getPosition(),
					                  world_.getApplet().color(0,255,0),8);
					world_.debugPoint(World.DEBUG_GRAPHGEN,
					                  road.getLaneChangeTarget(safe.getPosition(),newlane,
					                                           Car.LANE_CHANGE_ANGLE),
					                  world_.getApplet().color(255,0,0),6);
				}
			}
		}
	}

	private void addOpportunity2 ( RoadGraphNode node, int newlane, Car ahead,
	                               List<RoadGraphNode> targets ) {
		// in the current lane, immediately after passing each car in the adjacent
		// lane, until reaching the next car in the current lane
		// safe opportunity - car travels at max speed until braking just before
		// next car in lane
		// unsafe opportunity - car travels at max speed

		Road road = node.getRoad();
		int lane = road.getLane(node.getPosition());
		float time = node.getTime();

		for ( Car adjacent : world_.getCars() ) {
			// only consider cars in the desired adjacent lane
			if ( adjacent.getLane() != newlane ) {
				continue;
			}

			// if other car is going same speed or faster than we can go, can't pass
			if ( adjacent.getSpeed() >= car_.getMaxSpeed() ) {
				System.out.println("adjacent speed: [" + adjacent.getID() + "] "
				    + adjacent.getSpeed() + " " + car_.getMaxSpeed());
				continue;
			}

			// if other car is behind where we would change lanes to, it has already
			// been passed
			PVector target = road.getLaneChangeTarget(node.getPosition(),newlane,
			                                          car_.LANE_CHANGE_ANGLE);
			PVector threshold =
			    road.getAlong(target,-car_.getLength() - World.SPACING);
			world_.debugCircle(World.DEBUG_GRAPHGEN,threshold,
			                   world_.getApplet().color(255,255,255),6);
			if ( road.compareTo(adjacent.getFrontBumper(time),threshold) <= 0 ) {
				continue;
			}

			// compute passing point and time to get there at max speed
			float t = PVector.dist(target,
			                       road.getAlong(adjacent.getFrontBumper(time),
			                                     car_.getLength() + World.SPACING))
			    / (car_.getMaxSpeed() - adjacent.getSpeed());
			PVector p = road.getAlong(node.getPosition(),t * car_.getMaxSpeed());
			System.out.println("passing point: [" + adjacent.getID() + "]" + p + " "
			    + t + " / " + car_.getMaxSpeed() + " " + adjacent.getSpeed());

			// make sure there aren't other cars in the way
			List<Car> interval = world_
			    .getCarsInInterval(road,target,-car_.getLength() - World.SPACING,
			                       World.SPACING,time + t);
			if ( interval.size() > 1 ) {
				continue;
			}
			if ( interval.size() == 1 && !interval.contains(car_) ) {
				continue;
			}

			// unsafe opportunity - if that point is not ahead of the next car ahead
			// in this lane (or past the end of the road), create the opportunity
			// node
			if ( road.compareTo(p,road.getEnd(lane)) <= 0
			    && (ahead == null || road.compareTo(p,road
			        .getAlong(ahead.getRearBumper(time + t),-World.SPACING)) <= 0) ) {
				RoadGraphNode unsafe = new RoadGraphNode(road,p,time + t);
				targets.add(unsafe);
				world_.debugPoint(World.DEBUG_GRAPHGEN,unsafe.getPosition(),
				                  world_.getApplet().color(255,0,0),8);
				world_.debugPoint(World.DEBUG_GRAPHGEN,
				                  road.getLaneChangeTarget(unsafe.getPosition(),newlane,
				                                           car_.LANE_CHANGE_ANGLE),
				                  world_.getApplet().color(255,0,0),6);
				if ( ahead != null ) {
					world_.debugCircle(World.DEBUG_GRAPHGEN,
					                   road.getAlong(ahead.getCenter(time + t),0),
					                   ahead.getLength() / 2,ahead.getColor());
				}
				world_.debugCircle(World.DEBUG_GRAPHGEN,
				                   road.getAlong(adjacent.getCenter(time + t),0),
				                   adjacent.getLength() / 2,adjacent.getColor());
				System.out.println("unsafe: " + unsafe + " "
				    + road.getAlong(adjacent.getCenter(time + t),0));
			}

			if ( ahead == null ) {
				continue;
			}

			// compute point and time where braking starts
			float brakedist = car_.getBrakingDist(ahead.getSpeed());
			float braketime = car_.getBrakingTime(ahead.getSpeed());
			float tb = (braketime * car_.getMaxSpeed()
			    + PVector.dist(node.getPosition(),ahead.getRearBumper(node.getTime()))
			    - World.SPACING - brakedist)
			    / (car_.getMaxSpeed() - ahead.getSpeed());
			System.out.println(brakedist + " " + braketime + " " + tb);
			world_.debugPoint(World.DEBUG_GRAPHGEN,
			                  road.getAlong(car_.getCenter(time + tb),0),
			                  car_.getColor(),6);
			if ( tb < t ) {
				// safe opportunity - braking starts before desired passing point, so
				// need to take slowing into account
				float a = car_.getMaxAccel() / 2;
				float b = car_.getMaxSpeed() - adjacent.getSpeed();
				float c = (t - tb) * (car_.getMaxSpeed() - adjacent.getSpeed());
				if ( b * b - 4 * a * c < 0 ) {
					continue;
				}
				// time to passing point while braking
				float tp = -b + (float) Math.sqrt(b * b - 4 * a * c) / (2 * a);
				PVector ps = road.getAlong(node.getPosition(),t * car_.getMaxSpeed()
				    + car_.getMaxSpeed() * tp + car_.getMaxAccel() / 2 * tp * tp);
				// safe opportunity - if that point is not ahead of the next car ahead
				// in this lane (or past the end of the road), create the opportunity
				// node
				System.out
				    .println(time + " " + t + " " + tp + " / " + a + " " + b + " " + c);
				if ( road.compareTo(ps,road.getAlong(ahead.getRearBumper(time + t + tp),
				                                     -World.SPACING)) <= 0 ) {
					RoadGraphNode safe = new RoadGraphNode(road,p,time + t + tp);
					targets.add(safe);
					world_.debugPoint(World.DEBUG_GRAPHGEN,safe.getPosition(),
					                  world_.getApplet().color(0,255,0),8);
					world_.debugPoint(World.DEBUG_GRAPHGEN,
					                  road.getLaneChangeTarget(safe.getPosition(),newlane,
					                                           car_.LANE_CHANGE_ANGLE),
					                  world_.getApplet().color(255,0,0),6);
					world_.debugCircle(World.DEBUG_GRAPHGEN,
					                   road.getAlong(adjacent.getCenter(time + t + tp),0),
					                   ahead.getLength() / 2,ahead.getColor());
					System.out.println("safe: " + safe);
				}
			}
		}
	}

	private void addBoundary ( RoadGraphNode node, Car ahead,
	                           List<RoadGraphNode> targets ) {
		Road road = node.getRoad();
		int lane = road.getLane(node.getPosition());
		float time = node.getTime();

		// can't reach next car - no boundary node
		if ( car_.getMaxSpeed() <= ahead.getSpeed() ) {
			return;
		}
		/*
		 * // just drive forward at max speed to // the end of the road PVector end
		 * = road.getEnd(lane); if ( road.compareTo(node.getPosition(),end) < 0 ) {
		 * float t = PVector.dist(node.getPosition(),end) / car_.getMaxSpeed();
		 * RoadGraphNode boundary = new RoadGraphNode(road,end,time + t);
		 * targets.add(boundary);
		 * world_.debugPoint(World.DEBUG_GRAPHGEN,boundary.getPosition(),
		 * world_.getApplet().color(0,0,255),8); if ( ahead != null ) {
		 * world_.debugCircle(World.DEBUG_GRAPHGEN,
		 * road.getAlong(ahead.getCenter(time + t),0), ahead.getLength() /
		 * 3,ahead.getColor()); } System.out.println("boundary node: (a) " +
		 * boundary + " / " + car_.getMaxSpeed() + " " + (ahead == null ? ahead :
		 * ahead.getSpeed())); }
		 */
		// } else /* if ( ahead != null ) */ {
		// drive as fast as possible, then slow at last minute to max speed of
		// next car

		// time will be braking time + ( dist to boundary node - braking dist
		// )/max speed
		// dist to boundary node = dist to current ahead car position + ahead
		// car speed * time to boundary node
		// d = ad + as*t
		// t = bt + (d-bd)/ms = bt + (ad + as*t - bd)/ms
		// t*ms = bt*ms + ad +as*t - bd
		// t*(ms-as) = bt*ms + ad - bd
		// t = (bt*ms + ad - bd)/(ms-as)

		float t = getTimeToCatch(node,ahead,true);

		if ( t < .000001 ) { // we're already right behind the next car
			return;
		}

		// System.out.println(brakedist+" "+availdist);
		RoadGraphNode boundary = new RoadGraphNode(road,road
		    .getAlong(ahead.getRearBumper(time + t),-World.SPACING),time + t);
		targets.add(boundary);
		world_.debugPoint(World.DEBUG_GRAPHGEN,boundary.getPosition(),
		                  world_.getApplet().color(255,0,255),8);
		world_.debugCircle(World.DEBUG_GRAPHGEN,
		                   road.getAlong(ahead.getCenter(time + t),0),
		                   ahead.getLength() / 2,ahead.getColor());
		System.out.println("boundary node: (b) " + boundary);
	}

	private void addGoal ( RoadGraphNode node, Car ahead,
	                       List<RoadGraphNode> targets ) {
		// add goal if reachable - in the current lane and nothing in the way

		Road road = node.getRoad();
		int lane = road.getLane(node.getPosition());
		float time = node.getTime();

		if ( lane != road.getLane(goal_.getPoint()) ) {
			return;
		}

		float t =
		    PVector.dist(node.getPosition(),goal_.getPoint()) / car_.getMaxSpeed();

		if ( ahead == null || t <= getTimeToCatch(node,ahead,true) ) {
			RoadGraphNode goal = new RoadGraphNode(road,goal_.getPoint(),time + t);
			targets.add(goal);
			world_.debugPoint(World.DEBUG_GRAPHGEN,goal.getPosition(),
			                  world_.getApplet().color(0,255,0),8);
			System.out.println("goal node: " + goal);
		}

	}

	@Override
	public List<RoadGraphNode> getNextLocations ( RoadGraphNode node ) {
		List<RoadGraphNode> targets = new ArrayList<RoadGraphNode>();
		Road road = node.getRoad();
		int lane = road.getLane(node.getPosition());
		float time = node.getTime();

		// can't plan how other cars will change lanes, so expect next car ahead
		// to always be the one that is currently ahead
		Car ahead = world_.getNextCarInLane(road,node.getPosition());
		// System.out.println("ahead: " + ahead);

		// System.out.println("getNextLocations(): "+node);

		// goal node
		addGoal(node,ahead,targets);

		// boundary node - only add if there's a car ahead
		if ( ahead != null ) {
			addBoundary(node,ahead,targets);
		}

		// lane change nodes
		if ( lane > 0 ) {
			addLaneChange(node,lane - 1,targets);
		}
		if ( lane < road.getNumLanes() - 1 ) {
			addLaneChange(node,lane + 1,targets);
		}

		// opportunity nodes - in current lane, immediately after passing each
		// neighboring car, until next car in current lane
		if ( lane > 0 ) {
			addOpportunity(node,lane - 1,ahead,targets);
		}
		if ( lane < road.getNumLanes() - 1 ) {
			addOpportunity(node,lane + 1,ahead,targets);
		}

		// System.out.println(" ---------------- " + location + " " + car_);
		for ( RoadGraphNode target : targets ) {
			addEdge(new RoadGraphNode[] { node, target });
			// System.out.println("neighbors: " + edges_.get(edges_.size() - 1)[1]);
		}
		return targets;
	}

}