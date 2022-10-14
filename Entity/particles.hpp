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
			ks = p1->isStatic() ? 3000.0 : 1500.0;
			kd = 42.0;
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

			Vector2D deltaPos = p2->position() - p1->position(); 
			double distance = deltaPos.length();

			Vector2D deltaVel = p2->velocity() - p1->velocity();
			double dotvec = deltaPos.dot(deltaVel);
		
			double springForce = 100.0 * (distance - length) / distance;

			Vector2D spring = deltaPos.eigen() * springForce ;
			// spring += deltaVel * kd;

			p1->addForce(spring);
			p2->addForce(spring.negate());
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
				Particle *part = addParticle (pos.x(), pos.y(), 0.1);

				addSpring (prevPart, part, length);
				prevPart = part;
			}
			addSpring (prevPart, ball, length);
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
				bat->addAngularVelocity (-bat->angle());
				bat->scaleAngularVelocity (0.95);
			}
			for (int i = 0; i < balls.size(); i++) {
				Ball *ball = balls[i];
				ball->addAngle (ball->angularVelocity() * deltaTime);
				ball->addAngularVelocity (ball->torque() * deltaTime);
				ball->scaleAngularVelocity (1.0 - 0.2 * deltaTime); //angular drag

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
						//move out of contact
						float factor = ball1->radius() + ball2->radius() - dist.length();
						ball1->setPosition(ball1->position() + dist * factor / 2.0);
						ball2->setPosition(ball2->position() - dist * factor / 2.0);

						Vector2D relativeVelocity = ball1->velocity()- ball2->velocity();	 // - perprbp * bat->angularVelocity();

						Vector2D collisionNormal = dist;
						double J = -0.2 * (relativeVelocity.dot(collisionNormal)) /
									(collisionNormal.dot(collisionNormal));

						//apply impulse
						ball1->addVelocity(collisionNormal.eigen(), J / ball1->mass());
						ball2->addVelocity(collisionNormal.eigen(), -J / ball2->mass());
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

					Vector2D end1 = bat->endpoint1();
					Vector2D end2 = bat->endpoint2();
					Vector2D dist1 = end1 - bat->position(); //from center of bat to end
					Vector2D dist2 = end2 - bat->position();

					Vector2D toBallFromBatC = ball->position() - bat->position();
					onto1 = toBallFromBatC.projectOnto(dist1);
					onto2 = toBallFromBatC.projectOnto(dist2);

					Vector2D normal(cos(bat->angle()), sin(bat->angle()));
					double dist3 = toBallFromBatC.projectOnto(normal);
					double dist4 = toBallFromBatC.projectOnto(normal.perp());

					bool collision = false;
					Vector2D collisionPoint, collisionNormal;
					//ends
					if(onto1 > dist1.length()) {
						Vector2D toBall = ball->position() - end1;
						if (toBall.length() < ball->radius()) {
							collision = true;
							collisionPoint = end1;
							collisionNormal = toBall;
							// printf("Hit end 1\n");
						// de-collide - move out of the way
							if(toBall.length() - ball->radius() < 0.0)
								ball->setPosition (ball->position() - toBall * (toBall.length() - ball->radius()));
						}
					}
					else if(onto2 > dist2.length()) {
						Vector2D toBall = ball->position() - end2;
						if(toBall.length() < ball->radius()) {
							// printf("Hit end 2\n");
							collision = true;
							collisionPoint = end2;
							collisionNormal = toBall;

						// de-collide - move out of the way
							if(toBall.length() - ball->radius() < 0.0)
								ball->setPosition (ball->position() - toBall * (toBall.length() - ball->radius()));
						}
					} //side
					else if (ABS(dist3) < ball->radius()) {
						// printf("Hit center\n");
						collision = true;
						collisionPoint = bat->position();
						collisionNormal = toBallFromBatC;
						// de-collide - move out of the way
						ball->setPosition (ball->position() - normal * (dist3 - ball->radius()));
					}

					if(collision) {
						collisionNormal = collisionNormal.eigen();
						// Vector2D rap = collisionPoint - ball->position();
						// Vector2D rbp = collisionPoint - bat->position();
						
						Vector2D relativeVelocity = ball->velocity() - bat->velocity();	 // - perprbp * bat->angularVelocity();

						//why do we need such an odd coefficient?
						double J = 2.0 * relativeVelocity.dot(collisionNormal); // /
							// collisionNormal.dot(collisionNormal);

						// if(J < 0) continue;
						J /= 1.0 / ball->mass() + 1.0 / bat->mass();

						std::cout << "J: " << J << "\n";
						Vector2D Impulse = collisionNormal * J;
						//apply impulse
						std::cout << "Impulse : " << Impulse.toString() << "\n";

						ball->addVelocity(Impulse, -1.0 / ball->mass());
						bat->addVelocity(Impulse, 1.0 / bat->mass());
						ball->addTorque (20.0 * dist3 * SIGN(dist4) * J) ;// / ball->momentOfInertia());
						bat->addAngularVelocity (-10.0 * dist3 * SIGN(dist4) * J ) ; /// bat->momentOfInertia());
					}
				}
			}

		}
};
#endif
			// //balls against bats
			// for (int i = 0; i < balls.size(); i++) {
			// 	Ball *ball = balls[i];
			// 	for (int j = 0; j < bats.size(); j++) {
			// 		Bat *bat = bats[j];

			// 		Vector2D end1 = bat->endpoint1();
			// 		Vector2D end2 = bat->endpoint2();
			// 		Vector2D dist1 = end1 - bat->position(); //from center of bat to end
			// 		Vector2D dist2 = end2 - bat->position();

			// 		Vector2D toBallFromBatC = ball->position() - bat->position();
			// 		onto1 = toBallFromBatC.projectOnto(dist1);
			// 		onto2 = toBallFromBatC.projectOnto(dist2);

			// 		Vector2D normal(cos(bat->angle()), sin(bat->angle()));
			// 		double dist3 = toBallFromBatC.projectOnto(normal);

			// 		double dist4 = toBallFromBatC.projectOnto(normal.perp());

			// 		bool collision = false;
			// 		Vector2D collisionPoint, collisionNormal;
			// 		//ends
			// 		if(onto1 > dist1.length()) {
			// 			Vector2D toBall = ball->position() - end1;
			// 			if (toBall.length() < ball->radius()) {
			// 				collision = true;
			// 				collisionPoint = end1;
			// 				collisionNormal = toBall;
			// 				// bat->addTorque(dist3 * (20.0*M_PI/20.0*bat->mass()/deltaTime));
			// 				// ball->reflect(ball->position() - end1);
			// 				// ball->addTorque (dist4 * (20.0*M_PI/20.0*ball->mass()/deltaTime));
			// 				printf("Hit end 1\n");
			// 			}
			// 		}
			// 		else if(onto2 > dist2.length()) {
			// 			Vector2D toBall = ball->position() - end2;
			// 			if(toBall.length() < ball->radius()) {
			// 				printf("Hit end 2\n");
			// 				collision = true;
			// 				collisionPoint = end2;
			// 				collisionNormal = toBall;
			// 				// bat->addTorque(-dist3 * (20.0*M_PI/20.0*bat->mass()/deltaTime));
			// 				// ball->reflect(ball->position() - end2);
			// 				// ball->addTorque (dist4 * (20.0*M_PI/20.0*ball->mass()/deltaTime));
			// 			}
			// 		} //side
			// 		else if (ABS(dist3) < ball->radius()) {
			// 			printf("Hit center\n");
			// 			collision = true;
			// 			collisionPoint = bat->position();
			// 			collisionNormal = toBallFromBatC;
			// 			// ball->addForce (bat->force());
			// 			// ball->reflect (normal);
			// 				// ball->addTorque (dist4 * (20.0*M_PI/20.0*ball->mass()/deltaTime));
			// 		}

			// 		if(collision) {
			// 			collisionNormal = collisionNormal.eigen();
			// 			Vector2D rap = collisionPoint - ball->position();
			// 			Vector2D rbp = collisionPoint - bat->position();
			// 			double rapperp = rap.perpdot(collisionNormal);
			// 			double rbpperp = rbp.perpdot(collisionNormal);
				
			// 			const double restitution = 0.9;
			// 			Vector2D perprbp = rbp.perp();
			// 			Vector2D relativeVelocity = ball->velocity()- bat->velocity() - perprbp * bat->angularVelocity();
				
			// 			// double J = -(1 + restitution)*(relativeVelocity.dot(collisionNormal)) /
			// 			// 			(collisionNormal.dot(collisionNormal)) *
			// 			// 			(1/bat->mass() + 1/ball->mass()  + (rbpperp*rbpperp)/bat->momentOfInertia()); //+ sqr(rapperp)/myball.momentofinertia


			// 			double J = -5.0 * (relativeVelocity.dot(collisionNormal)) /
			// 						(collisionNormal.dot(collisionNormal));

			// 			//apply impulse
			// 			ball->addVelocity(collisionNormal.eigen(), J / ball->mass());
			// 			bat->addVelocity(collisionNormal.eigen(), -J / bat->mass());
			// 			ball->addAngularVelocity (-dist3 * J) ;// / ball->momentOfInertia());
			// 			bat->addAngularVelocity (-dist3 * J ) ; /// bat->momentOfInertia());
				
			// 	// 		//spin
			// 	// 		// Vector2D
			// 	// 		// vector2 batperp, velproj;
			// 	// 		// vector_copy(&mybat.normal, &batperp);
			// 	// 		// vector_perp(&batperp);
			// 	// 		// vector_project(&relativevel, &batperp, &velproj);
			// 	// 		// myball.angularvelocity -= 0.01*vector_length(&velproj)*j/myball.momentofinertia;
			// 		}
			// 	}
