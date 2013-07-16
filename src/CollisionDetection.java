

public class CollisionDetection {
	RigidBody2DApp app;
	int[] collide_indices;
	double[] collide_x, collide_y;
	double e;
	double[] verts_x, verts_y;

	public CollisionDetection(RigidBody2DApp app) {
		this.app = app;
		this.collide_x = new double[2];
		this.collide_y = new double[2];
		this.collide_indices = new int[2];
		this.verts_x = new double[4];
		this.verts_y = new double[4];
	}

	public double dotProduct(double v1_x, double v1_y, double v2_x, double v2_y) {
		return (v1_x * v2_x + v1_y * v2_y);
	}

	public double[] crossProduct(double a1, double a2, double a3, double b1,
			double b2, double b3) {
		double[] product = new double[3];
		product[0] = a2 * b3 - a3 * b2;
		product[1] = a3 * b1 - a1 * b3;
		product[2] = a1 * b2 - a2 * b1;
		return product;
	}

	// detects a collision between circle and rectangle
	public double[] detectCircleRect(int circle, int rect) {
		double[] norm = {0, 0, 0, -999, 0};
		
		double radius = this.app.body[circle].radius;

		double diff_x = this.app.body[circle].shape.getX()
				- this.app.body[rect].x1;
		double diff_y = this.app.body[circle].shape.getY()
				- this.app.body[rect].y1;
		double dist = Math.sqrt(diff_x * diff_x + diff_y * diff_y);
		
		// Intersection with four vertices
		if (dist <= radius) {
			//System.out.println("Collision vertex");
			double n_x = this.app.body[circle].shape.getX()
					- this.app.body[rect].x1;
			double n_y = this.app.body[circle].shape.getY()
					- this.app.body[rect].y1;
			double magnitude = Math.sqrt(n_x * n_x + n_y * n_y);

			
			norm[0] = n_x / magnitude;
			norm[1] = n_y / magnitude;
			norm[2] = 0.0;
			norm[3] = this.app.body[rect].x1;
			norm[4] = this.app.body[rect].y1;
			return norm;
		}
		diff_x = this.app.body[circle].shape.getX() - this.app.body[rect].x2;
		diff_y = this.app.body[circle].shape.getY() - this.app.body[rect].y2;
		dist = Math.sqrt(diff_x * diff_x + diff_y * diff_y);
		if (dist <= radius) {
			//System.out.println("Collision vertex");
			double n_x = this.app.body[circle].shape.getX()
					- this.app.body[rect].x2;
			double n_y = this.app.body[circle].shape.getY()
					- this.app.body[rect].y2;
			double magnitude = Math.sqrt(n_x * n_x + n_y * n_y);

			
			norm[0] = n_x / magnitude;
			norm[1] = n_y / magnitude;
			norm[2] = 0.0;
			norm[3] = this.app.body[rect].x2;
			norm[4] = this.app.body[rect].y2;
			return norm;
		}
		diff_x = this.app.body[circle].shape.getX() - this.app.body[rect].x3;
		diff_y = this.app.body[circle].shape.getY() - this.app.body[rect].y3;
		dist = Math.sqrt(diff_x * diff_x + diff_y * diff_y);
		if (dist <= radius) {
			//System.out.println("Collision vertex");
			double n_x = this.app.body[circle].shape.getX()
					- this.app.body[rect].x3;
			double n_y = this.app.body[circle].shape.getY()
					- this.app.body[rect].y3;
			double magnitude = Math.sqrt(n_x * n_x + n_y * n_y);

			
			norm[0] = n_x / magnitude;
			norm[1] = n_y / magnitude;
			norm[2] = 0.0;
			norm[3] = this.app.body[rect].x3;
			norm[4] = this.app.body[rect].y3;
			return norm;
		}
		diff_x = this.app.body[circle].shape.getX() - this.app.body[rect].x4;
		diff_y = this.app.body[circle].shape.getY() - this.app.body[rect].y4;
		dist = Math.sqrt(diff_x * diff_x + diff_y * diff_y);
		if (dist <= radius) {
			//System.out.println("Collision vertex");
			double n_x = this.app.body[circle].shape.getX()
					- this.app.body[rect].x4;
			double n_y = this.app.body[circle].shape.getY()
					- this.app.body[rect].y4;
			double magnitude = Math.sqrt(n_x * n_x + n_y * n_y);

			
			norm[0] = n_x / magnitude;
			norm[1] = n_y / magnitude;
			norm[2] = 0.0;
			norm[3] = this.app.body[rect].x4;
			norm[4] = this.app.body[rect].y4;

			return norm;
		}

		// Axis-Align the rectangle vertices and centre point of the circle
		double x1_new, x2_new, x3_new, x4_new;
		double y1_new, y2_new, y3_new, y4_new;
		double circle_x, circle_y;

		// Rotate both the vertices and the centre point of the circle by the
		// negative angle of rotation of the rectangle
		double phi = -this.app.body[rect].shape.getTheta();
		x1_new = Math.cos(phi) * this.app.body[rect].x1 - Math.sin(phi)
				* this.app.body[rect].y1;
		y1_new = Math.sin(phi) * this.app.body[rect].x1 + Math.cos(phi)
				* this.app.body[rect].y1;
		x2_new = Math.cos(phi) * this.app.body[rect].x2 - Math.sin(phi)
				* this.app.body[rect].y2;
		y2_new = Math.sin(phi) * this.app.body[rect].x2 + Math.cos(phi)
				* this.app.body[rect].y2;
		x3_new = Math.cos(phi) * this.app.body[rect].x3 - Math.sin(phi)
				* this.app.body[rect].y3;
		y3_new = Math.sin(phi) * this.app.body[rect].x3 + Math.cos(phi)
				* this.app.body[rect].y3;
		x4_new = Math.cos(phi) * this.app.body[rect].x4 - Math.sin(phi)
				* this.app.body[rect].y4;
		y4_new = Math.sin(phi) * this.app.body[rect].x4 + Math.cos(phi)
				* this.app.body[rect].y4;

		circle_x = Math.cos(phi) * this.app.body[circle].shape.getX()
				- Math.sin(phi) * this.app.body[circle].shape.getY();
		circle_y = Math.sin(phi) * this.app.body[circle].shape.getX()
				+ Math.cos(phi) * this.app.body[circle].shape.getY();

	
		// Test for collision with new axis aligned objects
		dist = distance(x1_new, y1_new, x2_new, y2_new, circle_x, circle_y);
		// Top side
		if (dist <= radius && circle_x <= x2_new && circle_x >= x1_new) {
			//System.out.println("Collision Top" + dist);

			double poi_x = circle_x;
			double poi_y = circle_y - radius;
			double poi_xt = Math.cos(-phi) * poi_x - Math.sin(-phi) * poi_y;
			double poi_yt = Math.sin(-phi) * poi_x + Math.cos(-phi) * poi_y;
			double n_x = this.app.body[circle].shape.getX() - poi_xt;
			double n_y = this.app.body[circle].shape.getY() - poi_yt;

			double magnitude = Math.sqrt(n_x * n_x + n_y * n_y);
			
			norm[0] = n_x / magnitude;
			norm[1] = n_y / magnitude;
			norm[2] = 0.0;
			norm[3] = poi_xt;
			norm[4] = poi_yt;
			return norm;
		}
		dist = distance(x2_new, y2_new, x3_new, y3_new, circle_x, circle_y);
		// Right side
		if (dist <= radius && circle_y <= y2_new && circle_y >= y3_new) {
			//System.out.println("Collision Right" + dist);
			double poi_x = circle_x - radius;
			double poi_y = circle_y;
			double poi_xt = Math.cos(-phi) * poi_x - Math.sin(-phi) * poi_y;
			double poi_yt = Math.sin(-phi) * poi_x + Math.cos(-phi) * poi_y;
			double n_x = this.app.body[circle].shape.getX() - poi_xt;
			double n_y = this.app.body[circle].shape.getY() - poi_yt;
			double magnitude = Math.sqrt(n_x * n_x + n_y * n_y);
			

			norm[0] = n_x / magnitude;
			norm[1] = n_y / magnitude;
			norm[2] = 0.0;
			norm[3] = poi_xt;
			norm[4] = poi_yt;
			return norm;
		}
		dist = distance(x3_new, y3_new, x4_new, y4_new, circle_x, circle_y);
		// Bottom side
		if (dist <= radius && circle_x <= x3_new && circle_x >= x4_new) {
			//System.out.println("Collision Bottom" + dist);
			double poi_x = circle_x;
			double poi_y = circle_y + radius;
			double poi_xt = Math.cos(-phi) * poi_x - Math.sin(-phi) * poi_y;
			double poi_yt = Math.sin(-phi) * poi_x + Math.cos(-phi) * poi_y;
			double n_x = this.app.body[circle].shape.getX() - poi_xt;
			double n_y = this.app.body[circle].shape.getY() - poi_yt;
			double magnitude = Math.sqrt(n_x * n_x + n_y * n_y);
			

			norm[0] = n_x / magnitude;
			norm[1] = n_y / magnitude;
			norm[2] = 0.0;
			norm[3] = poi_xt;
			norm[4] = poi_yt;
			return norm;
		}
		dist = distance(x4_new, y4_new, x1_new, y1_new, circle_x, circle_y);
		// Left Side
		if (dist <= radius && circle_y <= y1_new && circle_y >= y4_new) {
			//System.out.println("Collision Left" + dist);
			double poi_x = circle_x + radius;
			double poi_y = circle_y;
			double poi_xt = Math.cos(-phi) * poi_x - Math.sin(-phi) * poi_y;
			double poi_yt = Math.sin(-phi) * poi_x + Math.cos(-phi) * poi_y;
			double n_x = this.app.body[circle].shape.getX() - poi_xt;
			double n_y = this.app.body[circle].shape.getY() - poi_yt;
			double magnitude = Math.sqrt(n_x * n_x + n_y * n_y);
			

			norm[0] = n_x / magnitude;
			norm[1] = n_y / magnitude;
			norm[2] = 0.0;
			norm[3] = poi_xt;
			norm[4] = poi_yt;
			return norm;
		}
		return norm;

	}

	public double[] computeNormal() {

		int i = this.collide_indices[0];
		int next = i - 1;
		if (next < 0){
			next = 3;
		}
		int j = this.collide_indices[1];
		double v1_x = this.verts_x[j] - this.verts_x[i];
		double v1_y = this.verts_y[j] - this.verts_y[i];
		double v2_x = this.verts_x[next] - this.verts_x[i];
		double v2_y = this.verts_y[next] - this.verts_y[i];
		double[] u = crossProduct(v2_x, v2_y, 0, v1_x, v1_y, 0);
		double[] normal = crossProduct(u[0], u[1], u[2], v1_x, v1_y, 0);
		double magnitude = Math.sqrt(normal[0] * normal[0] + normal[1]
				* normal[1] + normal[2] * normal[2]);
		normal[0] = normal[0] / magnitude;
		normal[1] = normal[1] / magnitude;
		normal[2] = normal[2] / magnitude;

		return normal;
	}

	public Double[] handleCollisionWallPoly(int a, double[] n, double px,
			double py) {
		Double[] newStates = new Double[4];

		// Pre-compute values
		double v1a_x = this.app.state[2 + (a * 6)];
		double v1a_y = this.app.state[3 + (a * 6)];
		double rap_x = this.app.body[a].shape.getX() - px;
		double rap_y = this.app.body[a].shape.getY() - py;
		double wa = this.app.state[5 + (a * 6)];
		double[] crossP = crossProduct(0, 0, wa, rap_x, rap_y, 0);

		double v1ap_x = (-1.0 - e) * (v1a_x + crossP[0]);
		double v1ap_y = (-1.0 - e) * (v1a_y + crossP[1]);

		// Calculate numerator
		double numerator = dotProduct(v1ap_x, v1ap_y, n[0], n[1]);

		// Calculate denomenator
		double massa_inv = 1.0 / this.app.body[a].mass;

		double ia_inv = 1.0 / this.app.body[a].I;

		double[] prod = crossProduct(rap_x, rap_y, 0.0, n[0], n[1], 0.0);

		double term2 = dotProduct(prod[0], prod[1], prod[0], prod[1]);
		double denomenator = massa_inv + ia_inv * term2;

		double j = numerator / denomenator;

		// Calculate post-collision velocities of a
		newStates[0] = v1a_x + (j * n[0]) * massa_inv; // velocity of body x
		newStates[1] = v1a_y + (j * n[1]) * massa_inv; // velocity of body y

		double[] crossProd = crossProduct(rap_x, rap_y, 0.0, n[0], n[1], 0.0);

		newStates[2] = wa + crossProd[2] * ia_inv; // angular velocity of body

		newStates[3] = (double) a;

		return newStates;
	}

	// a - Rect with vertex in collision
	// b - Rect with edge in collision
	// n - normal to Rect b
	// px,py - vertex of a (point of collision)
	public Double[] handleCollisionPoly(int a, int b, double[] n, double px,
			double py) {
		Double[] newStates = new Double[8];

		// Pre-compute values
		double v1a_x = this.app.state[2 + (a * 6)];
		double v1a_y = this.app.state[3 + (a * 6)];

		double v1b_x = this.app.state[2 + (b * 6)];
		double v1b_y = this.app.state[3 + (b * 6)];

		double v1ab_x = (-1.0 - e) * (v1a_x - v1b_x);
		double v1ab_y = (-1.0 - e) * (v1a_y - v1b_y);

		double massa_inv = 1.0 / this.app.body[a].mass;
		double massb_inv = 1.0 / this.app.body[b].mass;

		double rap_x = this.app.body[a].shape.getX() - px;
		double rap_y = this.app.body[a].shape.getY() - py;
		double ia_inv = 1.0 / this.app.body[a].I;

		double rbp_x = this.app.body[b].shape.getX() - px;
		double rbp_y = this.app.body[b].shape.getY() - py;
		double ib_inv = 1.0 / this.app.body[b].I;

		// Calculate numerator
		double numerator = dotProduct(v1ab_x, v1ab_y, n[0], n[1]);

		// Calculate three summed terms of denominator
		double mass_sum = massa_inv + massb_inv;
		double term1 = dotProduct(n[0], n[1], n[0], n[1]) * mass_sum;

		double[] prod = crossProduct(rap_x, rap_y, 0.0, n[0], n[1], 0.0);

		double[] prod2 = crossProduct(prod[0], prod[1], prod[2], rap_x, rap_y,
				0.0);

		double term2 = dotProduct(ia_inv * prod2[0], ia_inv * prod2[1], n[0],
				n[1]);

		prod = crossProduct(rbp_x, rbp_y, 0.0, n[0], n[1], 0.0);
		prod2 = crossProduct(prod[0], prod[1], prod[2], rbp_x, rbp_y, 0.0);
		double term3 = dotProduct(ib_inv * prod2[0], ib_inv * prod2[1], n[0],
				n[1]);

		// Calculate impulse (j)
		double j = numerator / (term1 + term2 + term3);

		newStates[0] = v1a_x + (j * n[0] * massa_inv);
		newStates[1] = v1a_y + (j * n[1] * massa_inv);
		newStates[2] = v1b_x - (j * n[0] * massb_inv);
		newStates[3] = v1b_y - (j * n[1] * massb_inv);

		// Compute post collision angular velocities
		double wa = this.app.state[5 + (a * 6)];
		double wb = this.app.state[5 + (b * 6)];
		double[] crossProdA = crossProduct(rap_x, rap_y, 0, n[0], n[1], 0);
		double[] crossProdB = crossProduct(rbp_x, rbp_y, 0, n[0], n[1], 0);

		newStates[4] = wa + crossProdA[2] * ia_inv;
		newStates[5] = wb - crossProdB[2] * ib_inv;
		newStates[6] = (double) a;
		newStates[7] = (double) b;

		return newStates;

	}

	public double[] detectCircCirc(int circle1, int circle2) {

		double[] norm = { 0, 0, 0, -999, 0 };

		// Distance from center of circle 1 to center of circle 2
		double x_diff = this.app.body[circle1].shape.getX()
				- this.app.body[circle2].shape.getX();
		
		double y_diff = this.app.body[circle1].shape.getY()
				- this.app.body[circle2].shape.getY();

		double distance = Math.sqrt((x_diff * x_diff) + (y_diff * y_diff));

		if (distance <= this.app.body[circle1].radius
				+ this.app.body[circle2].radius) {

			double normal_x = this.app.body[circle2].shape.getX()
					- this.app.body[circle1].shape.getX();
			double normal_y = this.app.body[circle2].shape.getY()
					- this.app.body[circle1].shape.getY();
			double magnitude = Math.sqrt(normal_x * normal_x + normal_y
					* normal_y);

			normal_x = normal_x / magnitude;
			normal_y = normal_y / magnitude;
			norm[0] = normal_x;
			norm[1] = normal_y;
			norm[3] = 1;
		}

		return norm;
	}

	// Handles Collision circle to circle
	public Double[] handleCollisionCircle(int a, int b, double[] n) {
		Double[] newStates = new Double[8];

		// Pre-compute values
		double v1a_x = this.app.state[2 + (a * 6)];
		double v1a_y = this.app.state[3 + (a * 6)];

		double v1b_x = this.app.state[2 + (b * 6)];
		double v1b_y = this.app.state[3 + (b * 6)];

		double v1ab_x = (-1.0 - e) * (v1a_x - v1b_x);
		double v1ab_y = (-1.0 - e) * (v1a_y - v1b_y);

		double massa_inv = 1.0 / this.app.body[a].mass;
		double massb_inv = 1.0 / this.app.body[b].mass;

		// double rap_x = this.app.body[a].shape.getX() - px;
		// double rap_y = this.app.body[a].shape.getY() - py;
		double ia_inv = 1.0 / this.app.body[a].I;

		// double rbp_x = this.app.body[b].shape.getX() - px;
		// double rbp_y = this.app.body[b].shape.getY() - py;
		double ib_inv = 1.0 / this.app.body[b].I;

		// Calculate numerator
		double numerator = dotProduct(v1ab_x, v1ab_y, n[0], n[1]);

		// Calculate three summed terms of denominator
		double mass_sum = massa_inv + massb_inv;
		double term1 = dotProduct(n[0], n[1], n[0], n[1]) * mass_sum;

		// double[] prod = crossProduct(rap_x, rap_y, 0.0, n[0], n[1], 0.0);

		// double[] prod2 = crossProduct(prod[0], prod[1], prod[2], rap_x,
		// rap_y,
		// 0.0);

		double term2 = dotProduct(ia_inv * n[0], ia_inv * n[1], n[0], n[1]);

		// prod = crossProduct(rbp_x, rbp_y, 0.0, n[0], n[1], 0.0);
		// prod2 = crossProduct(prod[0], prod[1], prod[2], rbp_x, rbp_y, 0.0);
		double term3 = dotProduct(ib_inv * n[0], ib_inv * n[1], n[0], n[1]);

		// Calculate impulse (j)
		double j = numerator / (term1 + term2 + term3);

		newStates[0] = v1a_x + (j * n[0] * massa_inv);
		newStates[1] = v1a_y + (j * n[1] * massa_inv);
		newStates[2] = v1b_x - (j * n[0] * massb_inv);
		newStates[3] = v1b_y - (j * n[1] * massb_inv);

		newStates[4] = this.app.state[5 + (a * 6)];
		newStates[5] = this.app.state[5 + (b * 6)];
		newStates[6] = (double) a;
		newStates[7] = (double) b;

		return newStates;

	}

	// Detects Collision
	public double[] detectRectRect(int body1, int body2) {

		double[] norm1 = { 0, 0, 0, -999, 0};

		verts_x[0] = this.app.body[body2].x1;
		verts_x[1] = this.app.body[body2].x2;
		verts_x[2] = this.app.body[body2].x3;
		verts_x[3] = this.app.body[body2].x4;

		verts_y[0] = this.app.body[body2].y1;
		verts_y[1] = this.app.body[body2].y2;
		verts_y[2] = this.app.body[body2].y3;
		verts_y[3] = this.app.body[body2].y4;
		
		if (pointInPolygon(verts_x, verts_y, this.app.body[body1].x1,
				this.app.body[body1].y1)) {
			// System.out.println("Collision");
			double[] norm = computeNormal();
			norm1[0] = norm[0];
			norm1[1] = norm[1];
			norm1[2] = norm[2];
			norm1[3] = 1;

		}
		if (pointInPolygon(verts_x, verts_y, this.app.body[body1].x2,
				this.app.body[body1].y2)) {
			// System.out.println("Collision");
			double[] norm = computeNormal();
			norm1[0] = norm[0];
			norm1[1] = norm[1];
			norm1[2] = norm[2];
			norm1[3] = 2;

		}
		if (pointInPolygon(verts_x, verts_y, this.app.body[body1].x3,
				this.app.body[body1].y3)) {
			// System.out.println("Collision");
			double[] norm = computeNormal();
			norm1[0] = norm[0];
			norm1[1] = norm[1];
			norm1[2] = norm[2];
			norm1[3] = 3;

		}
		if (pointInPolygon(verts_x, verts_y, this.app.body[body1].x4,
				this.app.body[body1].y4)) {
			// System.out.println("Collision");
			double[] norm = computeNormal();
			norm1[0] = norm[0];
			norm1[1] = norm[1];
			norm1[2] = norm[2];
			norm1[3] = 4;

		}
		return norm1;

	}

	   /** Tests whether a given point (vertex) is inside a polygon
     *  @param px  The x-coordinate of the point to test
     *  @param py  The y-coordinate of the point to test
     *  @param x   The x coordinates of the n-sided polygon
     *  @param y   The y-coordinates of the n-sided polygon
     *  @return    True, if the point is on an edge or inside the polygon
     *             False, otherwise
     * */
	public boolean pointInPolygon(double[] x, double[] y, double px,
			double py) {
		boolean inPoly = false;
		int n = x.length;

		double minDist = 999999;
		int next = 0;

		for (int i = 0; i < n; i++) {
			next++;

			if (next == n)
				next = 0;
			if (y[i] <= py && y[next] >= py || y[next] <= py&& y[i] >= py) {
				double dist = distance(x[i], y[i], x[next], y[next], py, py);
				if (dist <= minDist) {
					this.collide_x[0] = x[i];
					this.collide_x[1] = x[next];
					this.collide_y[0] = y[i];
					this.collide_y[1] = y[next];
					this.collide_indices[0] = i;
					this.collide_indices[1] = next;
					minDist = dist;
				}
				double q = x[i] + (py - y[i]) / (y[next] - y[i]) * (x[next] - x[i]);
				if (q <= px) {
					inPoly = !inPoly;

				}
			}
		}

		return inPoly;
	}

	/**
	 * Finds the perpendicular distance between an extended line given by points
	 * (x1,y1), (x2,y2) and a point (m,n)
	 * @param  x1,y1  The first point that lies on the line
	 * @param  x2,y2  The second point that lies on the line	
	 * @param  m,n   The point lying on, above or below the line
	 * @return       The perpendicular distance 
	 */
	public double distance(double x1, double y1, double x2, double y2,
			double m, double n) {
		double A = y1 - y2;
		double B = x2 - x1;
		double C = (x1 * y2) - (x2 * y1);
		double numerator = Math.abs(A * m + B * n + C);
		double denomenator = Math.sqrt(A * A + B * B);
		double distance = numerator / denomenator;
		return distance;
	}

	public void findVertices(int i) {
		
		double xx = this.app.body[i].shape.getX() + this.app.body[i].width / 2;
		double yy = this.app.body[i].shape.getY() + this.app.body[i].height / 2;

		double x3 = this.app.body[i].shape.getX()
				+ ((xx - this.app.body[i].shape.getX()) * Math
						.cos(this.app.body[i].shape.getTheta()))
				+ (yy - this.app.body[i].shape.getY())
				* Math.sin(this.app.body[i].shape.getTheta());

		double y3 = this.app.body[i].shape.getY()
				+ ((xx - this.app.body[i].shape.getX()) * Math
						.sin(this.app.body[i].shape.getTheta()))
				- (yy - this.app.body[i].shape.getY())
				* Math.cos(this.app.body[i].shape.getTheta());

		this.app.body[i].x3 = x3;
		this.app.body[i].y3 = y3;

		xx = this.app.body[i].shape.getX() - this.app.body[i].width / 2;
		yy = this.app.body[i].shape.getY() - this.app.body[i].height / 2;

		double x1 = this.app.body[i].shape.getX()
				+ ((xx - this.app.body[i].shape.getX()) * Math
						.cos(this.app.body[i].shape.getTheta()))
				+ (yy - this.app.body[i].shape.getY())
				* Math.sin(this.app.body[i].shape.getTheta());
		double y1 = this.app.body[i].shape.getY()
				+ ((xx - this.app.body[i].shape.getX()) * Math
						.sin(this.app.body[i].shape.getTheta()))
				- (yy - this.app.body[i].shape.getY())
				* Math.cos(this.app.body[i].shape.getTheta());
		this.app.body[i].x1 = x1;
		this.app.body[i].y1 = y1;

		xx = this.app.body[i].shape.getX() - this.app.body[i].width / 2;
		yy = this.app.body[i].shape.getY() + this.app.body[i].height / 2;

		double x4 = this.app.body[i].shape.getX()
				+ ((xx - this.app.body[i].shape.getX()) * Math
						.cos(this.app.body[i].shape.getTheta()))
				+ (yy - this.app.body[i].shape.getY())
				* Math.sin(this.app.body[i].shape.getTheta());
		double y4 = this.app.body[i].shape.getY()
				+ ((xx - this.app.body[i].shape.getX()) * Math
						.sin(this.app.body[i].shape.getTheta()))
				- (yy - this.app.body[i].shape.getY())
				* Math.cos(this.app.body[i].shape.getTheta());
		this.app.body[i].x4 = x4;
		this.app.body[i].y4 = y4;

		xx = this.app.body[i].shape.getX() + this.app.body[i].width / 2;
		yy = this.app.body[i].shape.getY() - this.app.body[i].height / 2;

		double x2 = this.app.body[i].shape.getX()
				+ ((xx - this.app.body[i].shape.getX()) * Math
						.cos(this.app.body[i].shape.getTheta()))
				+ (yy - this.app.body[i].shape.getY())
				* Math.sin(this.app.body[i].shape.getTheta());
		double y2 = this.app.body[i].shape.getY()
				+ ((xx - this.app.body[i].shape.getX()) * Math
						.sin(this.app.body[i].shape.getTheta()))
				- (yy - this.app.body[i].shape.getY())
				* Math.cos(this.app.body[i].shape.getTheta());
		this.app.body[i].x2 = x2;
		this.app.body[i].y2 = y2;
	}

	/**
	 * 
	 * @param i
	 * @return {1,1} if both x and y are past the bounds {1, 0} if just x is
	 *         past the bounds {0, 1} if just y is past the bounds {0, 0}
	 *         otherwise
	 */
	public double[] detectWallCirc(int i) {
		double circle_x = this.app.body[i].shape.getX();
		double circle_y = this.app.body[i].shape.getY();
		double radius = this.app.body[i].radius;

		double[] result = { 0, 0, 0, 0, 0 ,0 };
		// Right side wall
		if (circle_x + radius >= this.app.xyBounds) {

			result[0] = 1;
		}
		// Left side wall
		if (circle_x - radius <= -this.app.xyBounds) {

			result[1] = 1;
		}
		// Top wall
		if (circle_y + radius >= this.app.xyBounds) {

			result[2] = 1;
		}
		// Bottom wall
		if (circle_y - radius <= -this.app.xyBounds) {

			result[3] = 1;
		}
		return result;

	}


	public double[] detectWallRect(int i) {
		findVertices(i);
		double x1 = this.app.body[i].x1;
		double x2 = this.app.body[i].x2;
		double x3 = this.app.body[i].x3;
		double x4 = this.app.body[i].x4;
		double y1 = this.app.body[i].y1;
		double y2 = this.app.body[i].y2;
		double y3 = this.app.body[i].y3;
		double y4 = this.app.body[i].y4;

		double[] result = { 0, 0, 0, 0, 0, 0 };
		// Right side wall
		if (x1 >= this.app.xyBounds){
			result[0] = 1;
			result[4] = x1;
			result[5] = y1;
			
		}else if (x2 >= this.app.xyBounds){
			result[0] = 1;
			result[4] = x2;
			result[5] = y2;
		}else if (x3 >= this.app.xyBounds){
			result[0] = 1;
			result[4] = x3;
			result[5] = y3;
		}else if (x4 >= this.app.xyBounds){
			result[0] = 1;
			result[4] = x4;
			result[5] = y4;
			// Left side wall
		} else if (x1 <= -this.app.xyBounds){
			result[1] = 1;
			result[4] = x1;
			result[5] = y1;
		}else if (x2 <= -this.app.xyBounds){
			result[1] = 1;
			result[4] = x2;
			result[5] = y2;
		}else if (x3 <= -this.app.xyBounds){
			result[1] = 1;
			result[4] = x3;
			result[5] = y3;
		}else if (x4 <= -this.app.xyBounds){
			result[1] = 1;
			result[4] = x4;
			result[5] = y4;
		// Top Wall
		} else if (y1 >= this.app.xyBounds){
			result[2] = 1;
			result[4] = x1;
			result[5] = y1;
			
		}else if (y2 >= this.app.xyBounds){
			result[2] = 1;
			result[4] = x2;
			result[5] = y2;
			
		}else if (y3 >= this.app.xyBounds){
			result[2] = 1;
			result[4] = x3;
			result[5] = y3;
			
		}else if (y4 >= this.app.xyBounds){
			result[2] = 1;
			result[4] = x4;
			result[5] = y4;
		
			// Bottom wall
		} else if (y1 <= -this.app.xyBounds){
			result[3] = 1;
			result[4] = x1;
			result[5] = y1;
			
		}else if (y2 <= -this.app.xyBounds){
			result[3] = 1;
			result[4] = x2;
			result[5] = y2;
			
		}else if (y3 <= -this.app.xyBounds){
			result[3] = 1;
			result[4] = x3;
			result[5] = y3;
			
		}else if (y4 <= -this.app.xyBounds){
			result[3] = 1;
			result[4] = x4;
			result[5] = y4;
		}
		return result;
	}

}
