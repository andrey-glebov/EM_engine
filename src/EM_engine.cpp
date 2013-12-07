#include "../include/EM_engine.h"

EM_engine::EM_engine()
{
    //ctor
    cycles = 0;
}

EM_engine::~EM_engine()
{
    //dtor
}

double EM_engine::sqr(double x){
    return x * x;
}

void EM_engine::calc_new(){
    cycles ++;
    double k = 1.0 / (4.0 * M_PI * eps * eps0);
    for(int pi = 0, ps = particles.size(); pi < ps; pi ++){
        particles[pi].ax = 0.0;
        particles[pi].ay = 0.0;
        for(int pj = 0; pj < ps; pj ++){
            if (pi == pj)
                continue;
            double dx = particles[pj].x - particles[pi].x;
            double dy = particles[pj].y - particles[pi].y;
            double r2 = sqr(dx) + sqr(dy);
            if (r2 < 1e-20)
                continue;
            double r = sqrt(r2);
            double E = - k * particles[pi].q * particles[pj].q / r2;
            double F = grav ? (G * particles[pi].m * particles[pj].m / r2) : 0.0;
            particles[pi].ax += (E + F) * (dx / r);
            particles[pi].ay += (E + F) * (dy / r);
        }
        particles[pi].ax /= particles[pi].m;
        particles[pi].ay /= particles[pi].m;
    }
    for(int pi = 0, ps = particles.size(); pi < ps; pi ++){
        particles[pi].x += particles[pi].vx * dt;
        particles[pi].y += particles[pi].vy * dt;
        particles[pi].vx += particles[pi].ax * dt;
        particles[pi].vy += particles[pi].ay * dt;
        //std::cout << "p" << pi << ": x= " << particles[pi].x << " y= " << particles[pi].y << std::endl;
    }
}

int EM_engine::add_part(particle *prt){
    if (prt -> m == 0.0)
        return -1;
    particles.push_back(*prt);
    return 0;
}
