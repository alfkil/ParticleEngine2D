#ifndef GAME_hpp
#define GAME_hpp

#include "../Entity/particles.hpp"
#include "../Border/renderer.hpp"

#include <vector>
#include <cstdlib>
#include <ctime>

class Game {
	private:
		ParticleSystem system;
        ParticleRenderer *renderer;

		Ball *ball;
	public:
		Game (ParticleRenderer *_renderer) :
            renderer(_renderer)
        {
			initialize();
		}

	private:
		void initialize() {
			//srand(time(0));

			ball = system.addBall (0.0, -2.0, //position
							10.0, //mass
							0.2 //radius
                            );			
			Stick *stick = system.addStick (0.0, 0.0, 0.1);
			system.addRope (stick, ball, 10); //number of joints on rope
			system.addBat (0.0, 1.0, 50.0 /*mass*/, M_PI/4.0 /*angle*/, 0.3 /*width*/);
		}

    public:
        void play() {
			renderer->setWorld(&system);

			system.setGravity(9.790); //earth
			double time = 0.0;
			double deltaTime = 0.005;

			while (time < 1000.0) {
				system.computeForces();
				if (time == 0.0)
					ball->addForce(Vector2D(20000.0, 0.0));

				system.animate(deltaTime);

	            renderer->ignite();
				time += deltaTime;
			}
        }
};
#endif
