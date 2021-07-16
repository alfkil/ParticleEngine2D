#ifndef GAME_hpp
#define GAME_hpp

#include "../Entity/particles.hpp"
#include "../Border/renderer.hpp"
#include "../Control/input.hpp"

#include <vector>
#include <cstdlib>
#include <ctime>

class Game {
	private:
		ParticleSystem system;
        ParticleRenderer *renderer;

		Ball *ball;
		Bat *bat;

		Input input;

	public:
		Game (ParticleRenderer *_renderer) :
            renderer(_renderer)
        {
			initialize();
		}

	private:
		void initialize() {
			//srand(time(0));

			ball = system.addBall (
							0.0, -2.0, //position
							0.2,	//mass
							0.2 	//radius
                            );
			//Stick *stick1 = system.addStick (0.0, 0.0, 0.1);
			//system.addRope (stick1, ball1, 8); //number of joints on rope

			// Ball *ball2 = system.addBall (
			// 				-1.0, -2.0, //position
			// 				7.0, 	//mass
			// 				0.25 	//radius
			// 				);
			// Stick *stick2 = system.addStick (0.0, 0.0, 0.1);
			// system.addRope (stick2, ball2, 8); //number of joints on rope

			// Ball *ball3 = system.addBall (2.0, -2.0, //position
			// 				8.0, 	//mass
			// 				0.3 	//radius
            //                 );
			// Stick *stick3 = system.addStick (0.0, 0.0, 0.1);
			// system.addRope (stick3, ball3, 8); //number of joints on rope

			bat = system.addBat (0.0, 1.0, 2.0 /*mass*/, 0.0 /*M_PI/2.0*/ /*angle*/, 0.3 /*width*/);
		}

    public:
        void play() {
			renderer->setWorld(&system);

			system.setGravity(0.0);//9.790); //earth
			double time = 0.0;
			double deltaTime = 0.005;

			while (time < 1000.0) {
				bat->addTorque (-bat->torque());
				ball->addTorque (-ball->torque());

				system.doCollisions(deltaTime);

				system.computeForces();
				// if (time == 0.0)
				// 	ball1->addForce(Vector2D(20000.0, 0.0));

				bat->addTorque(input.rotate() * (5.0/2.0*M_PI*bat->mass()/deltaTime));

				bat->move(input.deltaMouse() * bat->mass() * 10.0, deltaTime);
				bat->addDrag(deltaTime);

				system.animate(deltaTime);

	            renderer->ignite();
				time += deltaTime;
			}
        }
};
#endif
