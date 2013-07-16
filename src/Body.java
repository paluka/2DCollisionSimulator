


import org.opensourcephysics.display.DrawableShape;

public class Body {
	DrawableShape shape;
	int width;
	int height;
	int radius = -1;
	float mass, damping;
	double x1, x2, x3, x4, y1, y2, y3, y4; //Vertices
	double I;	//Inertia
	String id;
	//TODO
	//do elasticity double e;

	
	public Body(DrawableShape shape, int width, int height, float mass, float damping){
		this.shape = shape;
		this.width = width;
		this.height = height;
		this.mass = mass;
		this.damping = damping;
		this.I = (this.height * this.height + this.width * this.width)*mass/12;
		this.id = "rect";
		
	}
	
	public Body(DrawableShape shape, int diameter, float mass, float damping){
		this.shape = shape;
		this.radius = diameter/2;
		this.height = diameter;
		this.width = diameter;
		this.mass = mass;
		this.damping = damping;
		this.I = (this.mass*this.radius*this.radius)/4;
		this.id = "circ";

	}
}
