

import java.awt.Color;
import java.awt.Rectangle;
import java.util.Vector;

import javax.swing.JFrame;

import org.opensourcephysics.controls.AbstractSimulation;
import org.opensourcephysics.controls.SimulationControl;
import org.opensourcephysics.display.Dataset;
import org.opensourcephysics.display.DrawableShape;
import org.opensourcephysics.display.Trail;
import org.opensourcephysics.frames.PlotFrame;
import org.opensourcephysics.numerics.Euler;
import org.opensourcephysics.numerics.ODE;
import org.opensourcephysics.numerics.ODESolver;
import org.opensourcephysics.numerics.RK4;

/**
 * Rigid Body Collision Simulator in 2D
 * 
 * 
 * 
 * @date Fall 2011
 * @author Erik Paluka
 * 
 */
public class RigidBody2DApp extends AbstractSimulation implements ODE {

	ODESolver odeSolver = new RK4(this);
	PlotFrame frame = new PlotFrame("x", "y", "Rigid Body Collision Detection in 2D Application");
	PlotFrame eFrame = new PlotFrame("Time", "Total Energy", "Energy Plot");
	PlotFrame mFrame = new PlotFrame("Time", "Total Momentum", "Momentum Plot");

	CollisionDetection detect = new CollisionDetection(this);
	Vector<Double[]> wCollision = new Vector<Double[]>(); // Collision list
															// for walls
	Vector<Double[]> pCollision = new Vector<Double[]>(); // Collision list for
															// polygons
	//Trail[] trail;
	int trailcount = 0;
	int numRect;
	int numCirc;
	int N; // Number of bodies to simulate
	int xyBounds; // xyBounds

	double state[], stepSize, minStep, minVx, maxVx, minVy, maxVy, minD, maxD,
			minM, maxM, minAV, maxAV, time;
	Body body[];

	float maxBackSteps;

	double[] rightWall = { -1, 0 };
	double[] leftWall = { 1, 0 };
	double[] topWall = { 0, -1 };
	double[] botWall = { 0, 1 };

	/**
	 * Returns the state to the OSP framework
	 */
	@Override
	public double[] getState() {
		return state;
	}

	/**
	 * Sets the rate of change of the states
	 */
	@Override
	public void getRate(double[] state, double[] rate) {
		for (int i = 0; i < N; i++) {
			// Position
			rate[0 + (i * 6)] = state[2 + (i * 6)];
			rate[1 + (i * 6)] = state[3 + (i * 6)];
			// Velocity
			rate[2 + (i * 6)] = -body[i].damping * state[2 + (i * 6)]
					/ body[i].mass;
			rate[3 + (i * 6)] = -body[i].damping * state[3 + (i * 6)]
					/ body[i].mass;
			// Rotation
			rate[4 + (i * 6)] = state[5 + (i * 6)];
			rate[5 + (i * 6)] = -(body[i].damping * state[5 + (i * 6)])
					/ (body[i].mass * body[i].I);

		}
	}

	/**
	 * Steps the differential equation, settings each bodies new theta value,
	 * and X-Y coordinates. Also, does wall and body detection
	 */
	@Override
	protected void doStep() {
		// step the differential equation
		odeSolver.step();
		wCollision.clear();
		pCollision.clear();

		// Set bodies new x, y coordinates and theta value
		for (int i = 0; i < N; i++) {
			body[i].shape.setTheta(state[4 + (i * 6)]);
			body[i].shape.setXY(state[0 + (i * 6)], state[1 + (i * 6)]);
		}

		// Detect body wall collision and save the new states if a collision has occured
		for (int i = 0; i < N; i++) {
			double[] wall = { 0, 0, 0, 0, 0 , 5};
			
			if (body[i].id.equalsIgnoreCase("rect")) {
				wall = detect.detectWallRect(i);
			} else if (body[i].id.equalsIgnoreCase("circ")) {
				wall = detect.detectWallCirc(i);
			}

			if (wall[0] != 0 || wall[1] != 0 || wall[2] != 0 || wall[3] != 0) {
				Double[] newStates = null;
				
				if (body[i].id.equalsIgnoreCase("rect")) {
					if (wall[0] == 1) {
						System.out.println("Right Wall Collision");
						newStates = detect.handleCollisionWallPoly(i,
								rightWall, wall[4], wall[5]);
					} else if (wall[1] == 1) {
						System.out.println("Left Wall Collision");
						newStates = detect.handleCollisionWallPoly(i, leftWall,
								wall[4], wall[5]);
					} else if (wall[2] == 1) {
						System.out.println("Top Wall Collision");
						newStates = detect.handleCollisionWallPoly(i, topWall,
								wall[4], wall[5]);
					} else if (wall[3] == 1) {
						System.out.println("Bottom Wall Collision");
						newStates = detect.handleCollisionWallPoly(i, botWall,
								wall[4], wall[5]);
					}
				} else if (body[i].id.equalsIgnoreCase("circ")){
					if (wall[0] == 1) {
						System.out.println("Right Wall Collision");
						newStates = detect.handleCollisionWallPoly(i,
								rightWall, xyBounds, body[i].shape.getY());
					} else if (wall[1] == 1) {
						System.out.println("Left Wall Collision");
						newStates = detect.handleCollisionWallPoly(i, leftWall,
								-xyBounds, body[i].shape.getY());
					} else if (wall[2] == 1) {
						System.out.println("Top Wall Collision");
						newStates = detect.handleCollisionWallPoly(i, topWall,
								body[i].shape.getX(), xyBounds);
					} else if (wall[3] == 1) {
						System.out.println("Bottom Wall Collision");
						newStates = detect.handleCollisionWallPoly(i, botWall,
								body[i].shape.getX(), -xyBounds);
					}
				}
				wCollision.add(newStates);

			}

		}

		// Pairs of collision (body1 and body2)
		int[] pairs = new int[N];
		for (int i = 0; i < N; i++) {
			pairs[i] = -1;
		}

		// Detect polygon-polygon collision and save the new states
		for (int i = 0; i < N; i++) {
			for (int j = 0; j < N; j++) {
				if (i != j) {
					if (i != pairs[j]) {
						double[] norm = { 0, 0, 0, -999, 0 };

						// Rect rect detection
						if (body[i].id.equalsIgnoreCase("rect")
								&& body[j].id.equalsIgnoreCase("rect")) {
							norm = detect.detectRectRect(i, j);
						
						} else if (body[i].id.equalsIgnoreCase("circ")
								&& body[j].id.equalsIgnoreCase("circ")) {
							norm = detect.detectCircCirc(i, j);
						
						} else if (body[i].id.equalsIgnoreCase("circ")
								&& body[j].id.equalsIgnoreCase("rect")) {
							norm = detect.detectCircleRect(i, j);
						
						} else if (body[i].id.equalsIgnoreCase("rect")
								&& body[j].id.equalsIgnoreCase("circ")) {
							norm = detect.detectCircleRect(j, i);
						}

						if (norm[3] != -999) {
							Double[] newStates = null;

							if (body[i].id.equalsIgnoreCase("rect")
									&& body[j].id.equalsIgnoreCase("rect")) {
								if (norm[3] == 1) {
									newStates = detect.handleCollisionPoly(i,
											j, norm, body[i].x1, body[i].y1);
								} else if (norm[3] == 2) {
									newStates = detect.handleCollisionPoly(i,
											j, norm, body[i].x2, body[i].y2);
								} else if (norm[3] == 3) {
									newStates = detect.handleCollisionPoly(i,
											j, norm, body[i].x3, body[i].y3);
								} else if (norm[3] == 4) {
									newStates = detect.handleCollisionPoly(i,
											j, norm, body[i].x4, body[i].y4);
								}
							} else if (body[i].id.equalsIgnoreCase("circ")
									&& body[j].id.equalsIgnoreCase("circ")) {
								newStates = detect.handleCollisionCircle(i, j,
										norm);

							} else if (body[i].id.equalsIgnoreCase("circ")
									&& body[j].id.equalsIgnoreCase("rect")) {
								newStates = detect.handleCollisionPoly(i, j,
										norm, norm[3], norm[4]);
							} else if (body[i].id.equalsIgnoreCase("rect")
									&& body[j].id.equalsIgnoreCase("circ")) {
								newStates = detect.handleCollisionPoly(j, i,
										norm, norm[3], norm[4]);
							}
							pCollision.add(newStates);
							pairs[i] = j;
							System.out.println("Polygon-Polygon Collision");
						}
					}
				}
			}

		}
		for (Double[] w : wCollision) {
			int i = w[3].intValue();
			int counter = 0;
			double steps = 0;

			while (true) {
				counter++;

				odeSolver.setStepSize(-odeSolver.getStepSize());
				odeSolver.step();

				

				double newStep = (-odeSolver.getStepSize() / 2.0);
				if (newStep < minStep) {
					newStep = minStep;
				}
				odeSolver.setStepSize(newStep);
				odeSolver.step();
				body[i].shape.setTheta(state[4 + (i * 6)]);
				body[i].shape.setXY(state[0 + (i * 6)], state[1 + (i * 6)]);
				steps = odeSolver.getStepSize();

				double[] wall = { 0, 0, 0, 0, 0, 0};
				if (body[i].id.equalsIgnoreCase("rect")) {
					wall = detect.detectWallRect(i);
				} else if (body[i].id.equalsIgnoreCase("circ")) {
					wall = detect.detectWallCirc(i);
				}

				if ((wall[0] != 1 && wall[1] != 1 && wall[2] != 1 && wall[3] != 1)
						|| counter > maxBackSteps) {
					body[i].shape.setTheta(state[4 + (i * 6)]);
					body[i].shape.setXY(state[0 + (i * 6)], state[1 + (i * 6)]);
					state[2 + (i * 6)] = w[0];
					state[3 + (i * 6)] = w[1];

					state[5 + (i * 6)] = w[2];
					break;
				}
			}
			
			odeSolver.setStepSize(-steps);
			odeSolver.step();
			odeSolver.setStepSize(stepSize);
			odeSolver.step();
			body[i].shape.setTheta(state[4 + (i * 6)]);
			body[i].shape.setXY(state[0 + (i * 6)], state[1 + (i * 6)]);
		}

		for (Double[] q : pCollision) {

			double steps = 0;
			int counter = 0;
			
			while (true) {
				counter++;
				int i = q[6].intValue();
				int j = q[7].intValue();

				odeSolver.setStepSize(-odeSolver.getStepSize());
				odeSolver.step();

				

				double newStep = (-odeSolver.getStepSize() / 2.0);
				if (newStep < minStep) {
					newStep = minStep;
				}
				odeSolver.setStepSize(newStep);
				odeSolver.step();
				steps = odeSolver.getStepSize();
				body[i].shape.setTheta(state[4 + (i * 6)]);
				body[i].shape.setXY(state[0 + (i * 6)], state[1 + (i * 6)]);

				body[j].shape.setTheta(state[4 + (j * 6)]);
				body[j].shape.setXY(state[0 + (j * 6)], state[1 + (j * 6)]);

				double[] norm = { 0, 0, 0, -999, 0 };

				// Rect rect detection
				if (body[i].id.equalsIgnoreCase("rect")
						&& body[j].id.equalsIgnoreCase("rect")) {
					norm = detect.detectRectRect(i, j);

				} else if (body[i].id.equalsIgnoreCase("circ")
						&& body[j].id.equalsIgnoreCase("circ")) {
					norm  = detect.detectCircCirc(i, j);

				} else if (body[i].id.equalsIgnoreCase("circ")
						&& body[j].id.equalsIgnoreCase("rect")) {
					norm = detect.detectCircleRect(i, j);
				} else if (body[i].id.equalsIgnoreCase("rect")
						&& body[j].id.equalsIgnoreCase("circ")) {
					norm = detect.detectCircleRect(j, i);
				}

				if (norm[3] == -999 || counter > maxBackSteps) {

					body[i].shape.setTheta(state[4 + (i * 6)]);
					body[i].shape.setXY(state[0 + (i * 6)], state[1 + (i * 6)]);

					body[j].shape.setTheta(state[4 + (j * 6)]);
					body[j].shape.setXY(state[0 + (j * 6)], state[1 + (j * 6)]);
					int a = i;
					int b = j;

					state[2 + (a * 6)] = q[0];
					state[3 + (a * 6)] = q[1];
					state[2 + (b * 6)] = q[2];
					state[3 + (b * 6)] = q[3];

					state[5 + (a * 6)] = q[4];
					state[5 + (b * 6)] = q[5];
					break;
				}
			}
			
			odeSolver.setStepSize(-steps);
			odeSolver.step();
			odeSolver.setStepSize(stepSize);
			odeSolver.step();

		}
		
		double totEnergy = 0;
		double totM = 0;
		
		time += stepSize;
		
		for (int i = 0; i < N; i++) {
			//if (trailcount > 0) {
			//	trail[i].clear();
			//}
			//trail[i] = new Trail();
			//frame.addDrawable(trail[i]);

			body[i].shape.setTheta(state[4 + (i * 6)]);
			body[i].shape.setXY(state[0 + (i * 6)], state[1 + (i * 6)]);
			totEnergy += findEnergy(i);
			totM += findM(i);
			
			//trail[i].addPoint(state[0 + (i * 6)], state[1 + (i * 6)]);
			//trail[i].addPoint(state[2 + (i * 6)], state[3 + (i * 6)]);
			//trail[i].closeTrail();
		}
		eFrame.append(0, time, totEnergy);
		mFrame.append(0, time, totM);
		trailcount++;

	}
	
	public double findM(int i){
		Body b = body[i];
		
		double mT = state[2 + (i * 6)]*b.mass + state[3 + (i * 6)]*b.mass;
		
		double mR = b.I * state[5 + (i*6)];
		
		
		return mT + mR;
	}
	
	public double findEnergy(int i){
		Body b = body[i];
		
		double energyT = 0.5*b.mass;
		energyT *= detect.dotProduct(state[2 + (i * 6)], state[3 + (i * 6)], state[2 + (i * 6)], state[3 + (i * 6)]);
		
		double energyR = 0.5 * b.I * state[5 + (i*6)] * state[5 + (i*6)];
		return energyT + energyR;
	}
	
	/**
	 * Initializes the simulation
	 */
	@Override
	public void initialize() {
		wCollision.clear();
		pCollision.clear();
		time = 0;
		trailcount = 0;
		
		// initialize and set the step size for the ODE solver
		stepSize = control.getDouble("dt");
		odeSolver.initialize(stepSize);
		maxBackSteps = control.getInt("Max Back Steps");
		minStep = control.getDouble("Minimum Step Back Time");
		numCirc = control.getInt("Number of Circles");
		numRect = control.getInt("Number of Rectangles");
		minVx = control.getDouble("Minimum X Velocity");
		maxVx = control.getDouble("Maximum X Velocity");
		minVy = control.getDouble("Minimum Y Velocity");
		maxVy = control.getDouble("Maximum Y Velocity");
		minD = control.getDouble("Minimum Damping");
		maxD = control.getDouble("Maximum Damping");
		minM = control.getDouble("Minimum Mass");
		maxM = control.getDouble("Maximum Mass");
		minAV = control.getDouble("Minimum Angular Velocity");
		maxAV = control.getDouble("Maximum Angular Velocty");

		detect.e = control.getDouble("e");
		N = numCirc + numRect;
		xyBounds = control.getInt("XY bounds");
		
		frame.setPreferredMinMax(-xyBounds, xyBounds, -xyBounds, xyBounds);
		frame.clearDrawables();
		//frame.setAlwaysOnTop(true);
		frame.setLocation(0, 0);
		frame.setSize(control.getInt("Frame's size"),
				control.getInt("Frame's size"));
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		
		eFrame.setPreferredMinMax(-1, 5.0, -1000.0*N/2, 100000000.0*N);
		eFrame.setConnected(0,true);
		eFrame.setMarkerShape(0, Dataset.NO_MARKER);
		eFrame.setLocation(control.getInt("Frame's size"), 0);
		eFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		
		mFrame.setPreferredMinMax(-1, 5.0, -10000.0, 500000.0);
		mFrame.setConnected(0,true);
		mFrame.setMarkerShape(0, Dataset.NO_MARKER);
		mFrame.setLocation(control.getInt("Frame's size"), eFrame.getHeight());
		mFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		
		
		body = new Body[N];
		state = new double[N * 6];
		//trail = new Trail[N];

		for (int i = 0; i < N; i++) {

			state[2 + (i * 6)] = Math.random() * maxVx + minVx; // velocity x
			state[3 + (i * 6)] = Math.random() * maxVy + minVy; // velocity y
			state[4 + (i * 6)] = 0; // rotation angle. Keep it at 0 for proper
									// placement of polygons and circles
			state[5 + (i * 6)] = Math.random() * maxAV + minAV; // angular
																// velocity
			if (i < control.getInt("Number of Rectangles")) {
				placementRect(i);
			} else {
				placementCircle(i);
			}
		}
	}

	/**
	 * Creates a rectangular body, sets its colour, and adds it to the frame
	 * 
	 * @param i
	 *            - body index
	 */
	public void placementRect(int i) {
		int[] wh = place(i);

		DrawableShape shape = DrawableShape.createRectangle(state[0 + (i * 6)],
				state[1 + (i * 6)], wh[0], wh[1]);
		Color color = new Color((int) (Math.random() * 205 + 50),
				(int) (Math.random() * 205 + 50),
				(int) (Math.random() * 205 + 50));
		shape.setMarkerColor(color, color);
		float mass = (float) (Math.random() * maxM + minM);
		float damping = (float) (Math.random() * maxD + minD);
		body[i] = new Body(shape, wh[0], wh[1], mass, damping);
		frame.addDrawable(body[i].shape);
		body[i].shape.setTheta(state[4 + (i * 6)]);
		body[i].shape.setXY(state[0 + (i * 6)], state[1 + (i * 6)]);

	}

	/**
	 * Creates a circular body, sets its colour, and adds it to the frame
	 * 
	 * @param i
	 *            - body index
	 */
	public void placementCircle(int i) {
		int[] wh = place(i);

		DrawableShape shape = DrawableShape.createCircle(state[0 + (i * 6)],
				state[1 + (i * 6)], wh[0]);
		Color color = new Color((int) (Math.random() * 205 + 50),
				(int) (Math.random() * 205 + 50),
				(int) (Math.random() * 205 + 50));
		shape.setMarkerColor(color, color);
		float mass = (float) (Math.random() * maxM + minM);
		float damping = (float) (Math.random() * maxD + minD);
		
		body[i] = new Body(shape, wh[0], mass, damping);
		frame.addDrawable(body[i].shape);
		body[i].shape.setTheta(state[4 + (i * 6)]);
		body[i].shape.setXY(state[0 + (i * 6)], state[1 + (i * 6)]);

	}

	/**
	 * Randomly creates x and y position values, width and height values, and
	 * checks if a rectangle is already in that area. If it is, then create new
	 * values and try again
	 * 
	 * @param i
	 *            - body index
	 * @return int[2] - width and height
	 */
	public int[] place(int i) {
		boolean flag = false;
		int[] wh = new int[2];
		int count = 0;

		while (!flag) {
			boolean flag2 = false;
			// width
			wh[0] = (int) (Math.random()
					* ((Math.abs(xyBounds) + Math.abs(xyBounds)) / 5) + Math
					.abs(xyBounds) / 8);
			// height
			wh[1] = (int) (Math.random() * wh[0] / 2) + wh[0] / 2;
			state[0 + (i * 6)] = Math.random() * xyBounds - xyBounds + wh[0]; // position
																				// x
			state[1 + (i * 6)] = Math.random() * xyBounds - xyBounds + wh[1]; // position
																				// y

			int newlJava = (int) (state[0 + (i * 6)] - (wh[0] / 2));
			int newuJava = (int) (state[1 + (i * 6)] - (wh[1] / 2));

			// The body to place
			Rectangle rect1 = new Rectangle(newlJava, newuJava, wh[0] + 5,
					wh[1] + 5);

			// Iterate over the other bodies to see if the new one's
			// position overlaps with others
			for (int j = 0; j < i; j++) {
				// Find left upper coordinate of body
				int lJava = (int) (body[j].shape.getX() - (body[j].width / 2));
				int uJava = (int) (body[j].shape.getY() - (body[j].height / 2));

				Rectangle rect2 = new Rectangle(lJava, uJava,
						body[j].width + 5, body[j].height + 5);

				if (rect1.intersects(rect2)) {
					flag2 = true;
					count++;
					break;
				}

			}

			if (flag2 == false) {
				flag = true;
			}
			if (count > 10000000) {
				System.out
						.println("Error in placing the 2D bodies. N is possibly too large.");
				System.out.println("Ending program.");
				System.exit(1);
			}

		}

		return wh;
	}

	/**
	 * Resets the simulation
	 */
	@Override
	public void reset() {
		control.setValue("dt", 0.01);
		control.setValue("Number of Rectangles", 3);
		control.setValue("Number of Circles", 3);
		control.setValue("XY bounds", 50);
		control.setValue("Frame's size", 800);
		control.setValue("Max Back Steps", 2000);
		control.setValue("Minimum Step Back Time", 0.0000001);
		control.setValue("e", 1.0);
		control.setValue("Minimum X Velocity", -10);
		control.setValue("Maximum X Velocity", 20);
		control.setValue("Minimum Y Velocity", -10);
		control.setValue("Maximum Y Velocity", 20);
		control.setValue("Minimum Damping", 0);
		control.setValue("Maximum Damping", 1);
		control.setValue("Minimum Mass", 1);
		control.setValue("Maximum Mass", 10);
		control.setValue("Minimum Angular Velocity", 0);
		control.setValue("Maximum Angular Velocty", Math.PI / 2);
		
		trailcount = 0;
		initialize();
	}

	/**
	 * Starts the program
	 * 
	 * @param args
	 *            not used
	 */
	public static void main(String args[]) {
		SimulationControl.createApp(new RigidBody2DApp());
	}

}
