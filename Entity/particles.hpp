#ifndef PARTICLES_hpp
#define PARTICLES_hpp

#include "../Service/vector2d.hpp"

#include <vector>

using namespace std;

#define ABS(x) ((x) < 0.0f ? -(x) : (x))

int debugFlag = 0;
double onto1, onto2;

class Particle {
	private:
		double _mass;
		Vector2D pos, vel, _force;
		bool _isStatic, _hasGravity;

	public:
		Particle (double x, double y, double mass) :
			pos (x, y),
			vel (0.0, 0.0),
			_mass (mass),
			_force (0.0, 0.0)
		{
			setStatic (false);
			setGravity (true);
		}

		double mass() {
			return _mass;
		}

		Vector2D position() {
			return pos;
		}
		void setPosition(Vector2D newPos) {
			pos = newPos;
		}
		Vector2D velocity() {
			return vel;
		}
		void scaleVelocity(double f) {
			vel = vel * f;
		}
		void reflect(Vector2D normal) {
			normal = normal.eigen();
			vel += normal * vel.length();
			// vel = Vector2D(-vel.x(), -vel.y());
		}

		Vector2D force() {
			return _force;
		}
		double rawForce() {
			return sqrt(_force.x() * _force.x() + _force.y() * _force.y());
		}
		void addForce(Vector2D newForce) {
			_force += newForce;
		}

		void resetForce() {
			_force = Vector2D(0.0, 0.0);
		}

		void move(double deltaTime) {
			if (!_isStatic) {
				//take Euler step for now
				pos += vel * deltaTime;
				vel += _force * deltaTime;
			}
		}

		void setStatic (bool is) {
			_isStatic = is;
			if (is) {
				setGravity(false);
			}
		}

        bool isStatic () {
            return _isStatic;
        }

		void setGravity (bool has) {
			_hasGravity = has;
		}

        bool hasGravity () {
            return _hasGravity;
        }
		double distance (Vector2D d) {
			Vector2D rel = position() - d;
			return sqrt (rel.x()*rel.x() + rel.y() * rel.y());
		}
};

class Ball : public Particle {
	private:
		double _radius;
		double _angle;

	public:
		Ball (double x, double y, double mass, double radius) :
			Particle (x, y, mass),
			_radius(radius),
			_angle(M_PI/4.0)
		{		}

		double radius() {
			return _radius;
		}

		double angle() {
			return _angle;
		}
};

class Stick : public Particle {
	private:
		double _radius;

	public:
		Stick (double x, double y, double radius) :
			Particle (x, y, 0.0),
			_radius(radius)
		{
			setStatic (true);
		}

		double radius () {
			return _radius;
		}
};

class Spring {
	private:
		Particle *p1, *p2;
		double ks, kd;
		double length;
	
	public:
		Spring (Particle *_p1, Particle *_p2, double _length) :
			p1(_p1),
			p2(_p2),
			length(_length)
		{
			ks = p1->isStatic() ? 3000.0 : 1500.0;
			kd = 42.0;
		}

		Vector2D position1 () {
			return p1->position();
		}

		Vector2D position2 () {
			return p2->position();
		}

		void computeForce () {
			if(p1->position() == p2->position()) {
				return;
			}

			Vector2D deltaPos = p2->position() - p1->position(); 
			float distance = deltaPos.length();

			Vector2D deltaVel = p2->velocity() - p1->velocity();
			float dotvec = deltaPos.dot(deltaVel);
			
			float springForce = distance > length ? ks * (distance - length) / distance : 0.0;

			Vector2D spring = deltaPos.eigen() * springForce;
			spring += deltaVel * kd;

			p1->addForce(spring);
			p2->addForce(Vector2D(0.0, 0.0) - spring);
		}
};

class Bat : public Particle {
	private:
		double _angle;
		double _width;
		double _angularVelocity;
		double _momentOfInertia;
		double _torque;
		double _drag;
		const double restAngle = 0.0;
	public:
		Bat (double x, double y, double mass, double angle, double width) 
		:	Particle (x, y, mass),
			_angle(angle),
			_width(width),
			_drag(0.1)
		{ 
			_angularVelocity = 0.0;
			_torque = 0.0;
			_momentOfInertia = 1.0f / 1.0f * mass * width * width;
			setGravity (false);
		}

		double angle() {
			return _angle;
		}

		double width() {
			return _width;
		}

		void move (Vector2D deltap, double deltat) {
			Vector2D d = deltap / 100.0 * mass() / deltat;
			addForce (d);
		}

		double torque () {
			return _torque;
		}
		void addTorque (double added) {
			_torque += added;
		}

		void addDrag (double deltat) {
			scaleVelocity ( (1.0 - _drag) ); // / deltat);
		}

		void addAngle (double angle) {
			_angle += angle;
		}

		void addAngularVelocity (double avelo) {
			_angularVelocity += avelo;
		}

		void scaleAngularVelocity (double f) {
			_angularVelocity = _angularVelocity * f;
		}
		double angularVelocity() {
			return _angularVelocity;
		}

		double momentOfInertia() {
			return _momentOfInertia;
		}

		Vector2D endpoint1() {
			double a = _angle + M_PI / 2.0;
			return Vector2D (position().x() + cos(a) * _width / 2.0, position().y() + sin(a) * _width /2.0);
		}

		Vector2D endpoint2() {
			double a = _angle + M_PI / 2.0;
			return Vector2D (position().x() - cos(a) * _width / 2.0, position().y() - sin(a) * _width /2.0);
		}
};

class ParticleSystem {
	private:
		vector<Particle *> particles;
		vector<Spring *> springs;

		vector<Ball *> balls;
		vector<Stick *> sticks;
		vector<Bat *> bats;

		Vector2D gravity;

	public:
		ParticleSystem () {
			gravity = Vector2D(0.0, 0.0);
		}

		void setGravity(double _gravity) {
			gravity = Vector2D(0.0, _gravity);
		}

		Particle *addParticle (double x, double y, double mass) {
			Particle *part = new Particle (x, y, mass);
			particles.push_back (part);
			return part;
		}

		Ball *addBall(double x, double y, double mass, double radius) {
			Ball *ball = new Ball(x, y, mass, radius);
			particles.push_back(ball);
			balls.push_back(ball);
			return ball;
		}

		Stick *addStick (double x, double y, double radius) {
			Stick *stick = new Stick (x, y, radius);
			particles.push_back (stick);
			sticks.push_back(stick);
			return stick;
		}

		Bat *addBat (double x, double y, double mass, double angle, double width) {
			Bat *bat = new Bat (x, y, mass, angle, width);
			particles.push_back (bat);
			bats.push_back(bat);
			return bat;
		}

		void addSpring (Particle *p1, Particle *p2, double length) {
			springs.push_back( new Spring (p1, p2, length));
		}

		void addRope(Stick *stick, Ball *ball, int joints) {
			Vector2D pos(stick->position());
			Vector2D end(ball->position());
			Vector2D inc = (end - pos) / (joints + 1);
			double length = inc.length();

			Particle *prevPart = stick;
			for (int i = 0; i < joints; i++) {
				pos += inc;
				Particle *part = addParticle (pos.x(), pos.y(), 0.5);

				addSpring (prevPart, part, length);
				prevPart = part;
			}
			addSpring (prevPart, ball, length);
		}

		void computeForces() {
			for(int i = 0; i < particles.size(); i++) {
				Particle *part = particles[i];
				part->resetForce();
				if (part->hasGravity())
					part->addForce(gravity * part->mass());
			}

			for (int i = 0; i < springs.size(); i++) {
				Spring *spring = springs[i];
				spring->computeForce();
			}
		}

		void animate (double deltaTime) {
			for(int i = 0; i < particles.size(); i++) {
				Particle *part = particles[i];
				part->move(deltaTime);
			}
			for (int i = 0; i < bats.size(); i++) {
				Bat *bat = bats[i];
				bat->addAngle (bat->angularVelocity() * deltaTime);
				bat->addAngularVelocity (bat->torque() * deltaTime / bat->momentOfInertia());
				bat->addAngularVelocity (-bat->angle());
				bat->scaleAngularVelocity (0.95);
			}
		}

	public:
		vector<Spring *> &getSprings () {
			return springs;
		}

		vector<Ball *> &getBalls () {
			return balls;
		}

		vector<Stick *> &getSticks () {
			return sticks;
		}

		vector<Bat *> &getBats () {
			return bats;
		}
		void moveBat(Vector2D delta) {

		}
		void doCollisions (double deltaTime) {
			// balls against balls
			for (int i = 0; i < balls.size(); i++) {
				for (int j = i+1; j < balls.size(); j++) {
					double distancex = balls[i]->position().x() - balls[j]->position().x();
					double distancey = balls[i]->position().y() - balls[j]->position().y();
					double distance = sqrt(distancex*distancex + distancey*distancey);

					if(distance < balls[i]->radius() + balls[j]->radius()) {
						balls[i]->reflect(balls[i]->position() - balls[j]->position());
						balls[j]->reflect(balls[j]->position() - balls[i]->position());

						//move out of contact
						Vector2D dist(distancex, distancey);
						float factor = balls[i]->radius() + balls[j]->radius() - distance;
						balls[i]->setPosition(balls[i]->position() + dist * factor);
						balls[j]->setPosition(balls[j]->position() - dist * factor);
					}
				}
			}
			// ball against sticks

debugFlag = 0;
			//balls against bats
			for (int i = 0; i < balls.size(); i++) {
				Ball *ball = balls[i];
				for (int j = 0; j < bats.size(); j++) {
					Bat *bat = bats[j];

					Vector2D end1 = bat->endpoint1();
					Vector2D end2 = bat->endpoint2();
					Vector2D dist1 = end1 - bat->position(); //from center of bat to end
					Vector2D dist2 = end2 - bat->position();

					Vector2D toBallFromBatC = ball->position() - bat->position();
					onto1 = toBallFromBatC.projectOnto(dist1);
					onto2 = toBallFromBatC.projectOnto(dist2);

					Vector2D normal(cos(bat->angle()), sin(bat->angle()));
					double dist3 = toBallFromBatC.projectOnto(normal);

					//ends
					if(onto1 > dist1.length()) {
						Vector2D toBall = ball->position() - end1;
						if (toBall.length() < ball->radius()) {
							bat->addTorque(dist3 * (20.0*M_PI/20.0*bat->mass()/deltaTime));
							ball->reflect(ball->position() - end1);

							printf("Hit end 1\n");
						}
					}
					else if(onto2 > dist2.length()) {
						Vector2D toBall = ball->position() - end2;
						if(toBall.length() < ball->radius()) {
							printf("Hit end 2\n");
							bat->addTorque(-dist3 * (20.0*M_PI/20.0*bat->mass()/deltaTime));
							ball->reflect(ball->position() - end2);
						}
					} //side
					else if (ABS(dist3) < ball->radius()) {
						printf("Hit center\n");
						ball->addForce (bat->force());
						ball->reflect (normal);
					}
				}
			}

		}
};
#endif
