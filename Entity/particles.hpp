#ifndef PARTICLES_hpp
#define PARTICLES_hpp

#include "../Service/vector2d.hpp"
#include <math.h>

#include <vector>

#include <iostream>

using namespace std;

#define ABS(x) ((x) < 0.0f ? -(x) : (x))
#define SIGN(x) ((x) > 0.0 ? 1.0 : -1.0)
#define MIN(x,y) ((x) > (y) ? (y) : (x))
#define MAX(x,y) ((x) > (y) ? (x) : (y))

int debugFlag = 0;

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
			// normal = normal.eigen() * vel.length();
			// vel += normal * vel.length() * 1.7;

		    double velocityDotProduct = vel.dot(normal);
		    vel = vel - normal * 2.0 * velocityDotProduct; //Vector2D(vel.x() - 2 * velocityDotProduct * normal.x(), vel.y() - 2 * velocityDotProduct * normal.y());
			scaleVelocity(0.95);
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
		void addVelocity (Vector2D d, double f) {
			vel += d * f;
		}
		void resetForce() {
			_force = Vector2D(0.0, 0.0);
		}

		void move(double deltaTime) {
			if (!_isStatic) {
				//take Euler step for now
				pos += vel * deltaTime;
				vel += _force * 1.0 / _mass * deltaTime;
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
		void maxOut(double minX, double minY, double maxX, double maxY) {
			setPosition(Vector2D(MIN(MAX(pos.x(), minX), maxX), MIN(MAX(pos.y(), minY), maxY)));
		}
};

class Ball : public Particle {
	private:
		double _radius;
		double _angle;
		double _angularVelocity;
		double _momentOfInertia;
		double _torque;

	public:
		Ball (double x, double y, double mass, double radius) :
			Particle (x, y, mass),
			_radius(radius),
			_angle(M_PI/4.0)
		{
			_angularVelocity = 0.0;
			_torque = 0.0;
			_momentOfInertia = 1.0f / 1.0f * mass * radius * radius;
		}

		double radius() {
			return _radius;
		}

		double angle() {
			return _angle;
		}
		double torque () {
			return _torque;
		}
				void addTorque (double added) {
			_torque += added;
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
			ks = p1->isStatic() ? 1000.0 : 500.0;
			kd = p1->isStatic() ? 10.0f : 5.0f;
		}

		Vector2D position1 () {
			return p1->position();
		}

		Vector2D position2 () {
			return p2->position();
		}

		void computeForce (double deltaTime) {
			if(p1->position() == p2->position()) {
				return;
			}
                  
			Vector2D deltaPos = p1->position() - p2->position();
			double distance = deltaPos.length();

            // The Spring Force Is Added To The Force      
            Vector2D force;
			if(distance > length)
				force = (deltaPos.eigen()) * (distance - length) * ks;

			force += (p1->velocity() - p2->velocity()) * kd;

			// Vector2D deltaVel = p2->velocity() - p1->velocity();
			// double dotvec = deltaPos.dot(deltaVel);
		
			// double springForce = 20.0 * (distance - length) / distance;

			// Vector2D spring = deltaPos.eigen() * springForce ;
			// spring += deltaVel * kd;

			p1->addForce(force.negate());
			p2->addForce(force);
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

		Vector2D normal() {
			return Vector2D(cos(_angle), sin(_angle));
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

		void addRope(Stick *stick, Ball *ball, double length, int joints) {
			Vector2D pos(stick->position());
			Vector2D end(ball->position());
			Vector2D inc = (end - pos).eigen() * length / (joints + 1); //(end - pos) / (joints + 1);
			double len = inc.length();

			Particle *prevPart = stick;
			for (int i = 0; i < joints; i++) {
				pos += inc;
				Particle *part = addParticle (pos.x(), pos.y(), 0.05);

				addSpring (prevPart, part, len);
				prevPart = part;
			}
			addSpring (prevPart, ball, len);
		}

		void computeForces(double deltaTime) {
			for(int i = 0; i < particles.size(); i++) {
				Particle *part = particles[i];
				part->resetForce();
				if (part->hasGravity())
					part->addForce(gravity * part->mass());
			}

			for (int i = 0; i < springs.size(); i++) {
				Spring *spring = springs[i];
				spring->computeForce(deltaTime);
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
				bat->addAngularVelocity (bat->torque() * deltaTime);

				// -- add elastic drag to bat angle
				bat->addAngularVelocity (-bat->angle());
				bat->scaleAngularVelocity (0.95);
			}
			for (int i = 0; i < balls.size(); i++) {
				Ball *ball = balls[i];
				ball->addAngle (ball->angularVelocity() * deltaTime);
				ball->addAngularVelocity (ball->torque() * deltaTime);
				ball->scaleAngularVelocity (0.97); //1.0 - 0.3 * deltaTime); //angular drag

				ball->scaleVelocity(1.0 - 0.5 * deltaTime); //drag
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
					Ball *ball1 = balls[i];
					Ball *ball2 = balls[j];

					Vector2D dist = ball1->position() - ball2->position();

					if(dist.length() < ball1->radius() + ball2->radius()) {
						// move out of contact
						float factor = ball1->radius() + ball2->radius() - dist.length();
						ball1->setPosition(ball1->position() + dist * factor / 2.0);
						ball2->setPosition(ball2->position() - dist * factor / 2.0);

						// calculate relative velocity
						Vector2D relativeVelocity = ball1->velocity()- ball2->velocity();	 // - perprbp * bat->angularVelocity();

						// collision normal must be eigen vector
						Vector2D collisionNormal = dist.eigen();

						// calculate impulse
						double J = -2.0 * (relativeVelocity.dot(collisionNormal));
						J /= 1.0 / ball1->mass() + 1.0 / ball2->mass();

						Vector2D Impulse = collisionNormal * J;

						//apply impulse
						ball1->addVelocity(Impulse, 1.0 / ball1->mass());
						ball2->addVelocity(Impulse, -1.0 / ball2->mass());

						// apply torque
						double dist3 = ball1->velocity().projectOnto(collisionNormal.perp());
						double dist4 = ball1->velocity().projectOnto(collisionNormal);
						ball1->addTorque (dist3 * SIGN(dist4) * J) ;// / ball->momentOfInertia());
						ball2->addTorque (-1.0 * dist3 * SIGN(dist4) * J) ;// / ball->momentOfInertia());
					}
				}
			}
			// ball against sticks

			//balls against sides
			for (int i = 0; i < balls.size(); i++) {
				Ball *ball = balls[i];

				if (ball->position().x() > 3.2) {
					ball->reflect(Vector2D(-1.0, 0.0));
				}
				if(ball->position().x() < -3.2) {
					ball->reflect(Vector2D(1.0, 0.0));
				}
				if(ball->position().y() > 2.4) {
					ball->reflect(Vector2D(0.0, -1.0));
				}
				if(ball->position().y() < -2.4) {
					ball->reflect(Vector2D(0.0, 1.0));
				}
				ball->maxOut(-3.2, -2.4, 3.2, 2.4);
			}
			//balls against bats
			for (int i = 0; i < balls.size(); i++) {
				Ball *ball = balls[i];
				for (int j = 0; j < bats.size(); j++) {
					Bat *bat = bats[j];

					Vector2D dist = ball->position() - bat->position();

					double projNorm = dist.projectOnto(bat->normal());
					double projPerpNorm = dist.projectOnto(bat->normal().perp());

					bool collision = false;
					Vector2D collisionPoint, collisionNormal;

					// note : test for three intervals
					// (to the left of bat, to the right of bat, within range of middle of bat)

					// test for collision with end1
					if(projPerpNorm < -bat->width()/2.0) {
						Vector2D toBall = ball->position() - bat->endpoint1();
						if (toBall.length() < ball->radius()) {
							collision = true;
							collisionPoint = bat->endpoint1();;
							collisionNormal = toBall;

							// move out of contact
							if(toBall.length() - ball->radius() < 0.0)
								ball->setPosition (ball->position() - toBall * (toBall.length() - ball->radius()));
						}
					}
					// test for collision with end2
					else if(projPerpNorm > bat->width()/2.0) {
						Vector2D toBall = ball->position() - bat->endpoint2();
						if(toBall.length() < ball->radius()) {
							collision = true;
							collisionPoint = bat->endpoint2();
							collisionNormal = toBall;

							// move out of contact
							if(toBall.length() - ball->radius() < 0.0)
								ball->setPosition (ball->position() - toBall * (toBall.length() - ball->radius()));
						}
					}
					// test for collision against side
					else if (ABS(projNorm) < ball->radius()) {
						collision = true;
						collisionPoint = bat->position();
						collisionNormal = bat->normal() * SIGN(projNorm);

						// move out of contact
						ball->setPosition (ball->position() - collisionNormal * (ABS(projNorm) - ball->radius()));
					}

					if(collision) {
						// collision normal must be eigen vector
						collisionNormal = collisionNormal.eigen();
						
						// calculate relative velocity
						Vector2D relativeVelocity = bat->velocity() - ball->velocity();

						// coefficient 1.0 to 2.0 is inelastic to elastic collision
						double J = -1.8 * relativeVelocity.dot(collisionNormal);
						J /= 1.0 / ball->mass() + 1.0 / bat->mass();

						// calculate impulse
						Vector2D Impulse = collisionNormal * J;

						// apply impulse
						ball->addVelocity(Impulse, -1.0 / ball->mass());
						bat->addVelocity(Impulse, 1.0 / bat->mass());

						// apply torque
						ball->addTorque (20.0 * projNorm * SIGN(projPerpNorm) * J) ;// / ball->momentOfInertia());
						bat->addAngularVelocity (-10.0 * projNorm * SIGN(projPerpNorm) * J ) ; /// bat->momentOfInertia());
					}
				}
			}

		}
};
#endif