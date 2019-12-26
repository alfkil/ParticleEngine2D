#ifndef PARTICLES_hpp
#define PARTICLES_hpp

#include "../Service/vector2d.hpp"

#include <vector>

using namespace std;

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

		Vector2D velocity() {
			return vel;
		}

		Vector2D force() {
			return _force;
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
		{ }

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

	public:
		Bat (double x, double y, double mass, double angle, double width) 
		:	Particle (x, y, mass),
			_angle(angle),
			_width(width)
		{ 
			setGravity (false);
		}

		double angle() {
			return _angle;
		}

		double width() {
			return _width;
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
};
#endif
