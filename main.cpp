#include "Border/renderer_glut.hpp"
#include "Control/game.hpp"

#include <unistd.h>

int main(int argc, char **argv) {
    GLUTParticleRenderer renderer(argc, argv);
    Game game(&renderer);
    game.play();

    return 0;
}
