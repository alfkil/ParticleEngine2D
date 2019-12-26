#ifndef PARTICLE_RENDERER_hpp
#define PARTICLE_RENDERER_hpp

#include "../Entity/particles.hpp"

class ParticleRenderer {
    public:
        virtual void ignite() = 0;
        virtual void setWorld(ParticleSystem *world) = 0;
};

#endif
