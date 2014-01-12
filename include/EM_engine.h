#ifndef EM_ENGINE_H
#define EM_ENGINE_H

#include <cmath>
#include <iostream>
#include <vector>

#define dt 1e-3
#define eps0 8.85e-12
#define eps 1e0
#define G 6.67e-11
#define re 5e-3
#define dist_m 1e1

#define USE_GRAV 0x01
#define USE_EL 0x02

#define USE_SPRNG 0x08
#define USE_RK 0x10
#define USE_QT 0x20

#define USE_DUMP 0x80

double sqr(double x);

const double k_el = 1.0 / (4.0 * M_PI * eps * eps0);

struct particle{
    double q, m;
    double x, y;
    double vx, vy;
    double ax, ay;
    double cur_x, cur_y, cur_vx, cur_vy;
    double k1_x, k2_x, k3_x, k4_x;
    double k1_y, k2_y, k3_y, k4_y;
    double k1_vx, k2_vx, k3_vx, k4_vx;
    double k1_vy, k2_vy, k3_vy, k4_vy;

    unsigned int color;
    double rm;

    bool is_stat;
};

struct spring{
    int p1, p2;
    double k;
    double l0;
};

struct qt_node{
    qt_node *nodes[4];
    double x1, y1, x2, y2, xc, yc;

    int part_idx;
    double qs, ms;
    double qcx, qcy, mcx, mcy;

    bool is_leaf;
};

class quad_tree{
    public:
        quad_tree(std::vector <particle> &parts, bool uel, bool ugrav);
        ~quad_tree();
        void get_res(std::vector <particle> &parts_new);

    private:
        qt_node *tree;
        std::vector <particle> prt;

        bool is_in_range(int idx, qt_node *cur_node);
        double dist(int idx, qt_node *cur_node);
        void add_part(int idx, qt_node *cur_node, int depth);
        void calc_one(int idx, qt_node *cur_node, int depth);
        void clear(qt_node *cur_node);

        double cur_ax, cur_ay;

        bool ug, ue;
};

class EM_engine
{
    public:
        EM_engine(int mode);
        ~EM_engine();

        std::vector <particle> particles;
        std::vector <spring> springs;
        quad_tree *qt;

        int cycles;

        void calc_spr();
        void calc_accel();
        void calc_accel_qt();

        void calc_next();

        int add_part(particle *prt);
        int del_part(int idx);

        int add_spring(spring *spr);
        int del_spring(int p1, int p2);

    protected:
    private:
        void integr_euler();
        void integr_rk();
        void (EM_engine::*integr_method)(void);
        void (EM_engine::*calc_method)(void);
        void dump();
        bool ugrav, uel, uspr, urk, uqt, dmp;
};

#endif // EM_ENGINE_H
