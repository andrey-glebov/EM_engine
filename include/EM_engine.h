#ifndef EM_ENGINE_H
#define EM_ENGINE_H

#include <cmath>
#include <iostream>
#include <vector>

#define dt 1e-3
#define eps0 8.85e-12
#define eps 1e-3
#define G 6.67e-11

#define grav true

struct particle{
    double q, m;
    double x, y;
    double vx, vy;
    double ax, ay;
};

class EM_engine
{
    public:
        EM_engine();
        virtual ~EM_engine();
        std::vector <particle> particles;
        double sqr(double x);
        void calc_new();
        int add_part(particle *prt);
    protected:
    private:
        int cycles;
};

#endif // EM_ENGINE_H
