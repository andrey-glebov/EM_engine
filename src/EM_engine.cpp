#include <algorithm>

#include "../include/EM_engine.h"

double sqr(double x){
    return x * x;
}

quad_tree::quad_tree(std::vector <particle> &parts, bool uel, bool ugrav){
    ue = uel;
    ug = ugrav;

    prt.assign(parts.begin(), parts.end());
    tree = new qt_node;
    tree -> is_leaf = true;
    tree -> part_idx = -1;
    for(int i = 0; i < 4; i ++)
        tree -> nodes[i] = NULL;

    double maxx = prt[0].x , maxy = prt[0].y, minx = prt[0].x, miny = prt[0].y;
    for(int i = 1, l = prt.size(); i < l; i ++){
        if (prt[i].x > maxx)
            maxx = prt[i].x;
        if (prt[i].y > maxy)
            maxy = prt[i].y;
        if (prt[i].x < minx)
            minx = prt[i].x;
        if (prt[i].y < miny)
            miny = prt[i].y;
    }

    tree -> x1 = minx;
    tree -> y1 = miny;
    tree -> x2 = maxx;
    tree -> y2 = maxy;
    tree -> xc = (minx + maxx) / 2.0;
    tree -> yc = (miny + maxy) / 2.0;

    for(int i = prt.size() - 1; i >= 0; i --)
        add_part(i, tree, 0);
}

quad_tree::~quad_tree(){
    clear(tree);
    delete tree;
}

void quad_tree::get_res(std::vector <particle> &parts_new){
    parts_new.assign(prt.begin(), prt.end());
    for(int i = parts_new.size(); i >= 0; i --){
        if (prt[i].is_stat)
            continue;
        cur_ax = 0.0;
        cur_ay = 0.0;
        calc_one(i, tree, 0);
        parts_new[i].ax = cur_ax / parts_new[i].m;
        parts_new[i].ay = cur_ay / parts_new[i].m;
    }
}

bool quad_tree::is_in_range(int idx, qt_node *cur_node){
    /*std::cout << "range: p" << idx << ": x=" << prt[idx].x << " y=" << prt[idx].y << std::endl;
    std::cout << "range: n:" << cur_node -> x1 << " " << cur_node -> y1 << " " \
              << cur_node -> x2 << " " << cur_node -> y2 << " " \
              << cur_node -> xc << " " << cur_node -> yc << std::endl;*/
    return ((prt[idx].x >= cur_node -> x1) && (prt[idx].x <= cur_node -> x2) &&
            (prt[idx].y >= cur_node -> y1) && (prt[idx].y <= cur_node -> y2));
}

double quad_tree::dist(int idx, qt_node *cur_node){
    if (is_in_range(idx, cur_node))
        return 0.0;

    if ((cur_node -> x1 < prt[idx].x) && (prt[idx].x < cur_node -> x2))
        return std::min(prt[idx].x - cur_node -> x1, cur_node -> x2 - prt[idx].x);
    if ((cur_node -> y1 < prt[idx].y) && (prt[idx].y < cur_node -> y2))
        return std::min(prt[idx].y - cur_node -> y1, cur_node -> y2 - prt[idx].y);

    if ((prt[idx].x < cur_node -> x1) && (prt[idx].y < cur_node -> y1))
        return sqrt(sqr(prt[idx].x - cur_node -> x1) + sqr(prt[idx].y - cur_node -> y1));
    if ((prt[idx].x < cur_node -> x1) && (prt[idx].y > cur_node -> y2))
        return sqrt(sqr(prt[idx].x - cur_node -> x1) + sqr(prt[idx].y - cur_node -> y2));
    if ((prt[idx].x > cur_node -> x2) && (prt[idx].y < cur_node -> y1))
        return sqrt(sqr(prt[idx].x - cur_node -> x2) + sqr(prt[idx].y - cur_node -> y1));
    if ((prt[idx].x > cur_node -> x2) && (prt[idx].y > cur_node -> y2))
        return sqrt(sqr(prt[idx].x - cur_node -> x2) + sqr(prt[idx].y - cur_node -> y2));

    return -1.0;
}

void quad_tree::add_part(int idx, qt_node *cur_node, int depth){
    /*std::cout << idx << " " << (void *) cur_node << " " << depth << std::endl;
    std::cout << cur_node -> x1 << " " << cur_node -> y1 << " " \
              << cur_node -> x2 << " " << cur_node -> y2 << " " \
              << cur_node -> xc << " " << cur_node -> yc << std::endl;*/
    if(cur_node -> part_idx == -1){
        cur_node -> part_idx = idx;
        cur_node -> qs = prt[idx].q;
        cur_node -> ms = prt[idx].m;
        cur_node -> qcx = prt[idx].x;
        cur_node -> qcy = prt[idx].y;
        cur_node -> mcx = prt[idx].x;
        cur_node -> mcy = prt[idx].y;
        //std::cout << "add: case new\n";
        return;
    }
    if (cur_node -> is_leaf){
        for(int i = 0; i < 4; i ++){
            cur_node -> nodes[i] = new qt_node;
            cur_node -> nodes[i] -> x1 = ((i & 0b01) ? (cur_node -> xc) : (cur_node -> x1));
            cur_node -> nodes[i] -> x2 = ((i & 0b01) ? (cur_node -> x2) : (cur_node -> xc));
            cur_node -> nodes[i] -> y1 = ((i & 0b10) ? (cur_node -> yc) : (cur_node -> y1));
            cur_node -> nodes[i] -> y2 = ((i & 0b10) ? (cur_node -> y2) : (cur_node -> yc));
            cur_node -> nodes[i] -> xc = (cur_node -> nodes[i] -> x1 + cur_node -> nodes[i] -> x2) / 2.0;
            cur_node -> nodes[i] -> yc = (cur_node -> nodes[i] -> y1 + cur_node -> nodes[i] -> y2) / 2.0;
            cur_node -> nodes[i] -> is_leaf = true;
            cur_node -> nodes[i] -> part_idx = -1;
        }
        cur_node -> is_leaf = false;
        //std::cout << "add: created\n";
    }
    for(int i = 0; i < 4; i ++)
        if(is_in_range(idx, cur_node -> nodes[i])){
            //std::cout << "part " << idx << " put at node " << i << " depth " << depth + 1 << std::endl;
            add_part(idx, cur_node -> nodes[i], depth + 1);
            break;
        }
    cur_node -> qs = 0.0;
    cur_node -> ms = 0.0;
    cur_node -> qcx = 0.0;
    cur_node -> qcy = 0.0;
    cur_node -> mcx = 0.0;
    cur_node -> mcy = 0.0;
    for(int i = 0; i < 4; i ++){
        if ((cur_node -> nodes[i] -> is_leaf == false) || (cur_node -> nodes[i] -> part_idx != -1)){
            cur_node -> qs += cur_node -> nodes[i] -> qs;
            cur_node -> ms += cur_node -> nodes[i] -> ms;
            cur_node -> qcx += cur_node -> nodes[i] -> qs * cur_node -> nodes[i] -> qcx;
            cur_node -> qcy += cur_node -> nodes[i] -> qs * cur_node -> nodes[i] -> qcy;
            cur_node -> mcx += cur_node -> nodes[i] -> ms * cur_node -> nodes[i] -> mcx;
            cur_node -> mcy += cur_node -> nodes[i] -> ms * cur_node -> nodes[i] -> mcy;
        }
    }
    if (cur_node -> part_idx != -1){
        cur_node -> qs += prt[cur_node -> part_idx].q;
        cur_node -> ms += prt[cur_node -> part_idx].m;
        cur_node -> qcx += prt[cur_node -> part_idx].x * prt[cur_node -> part_idx].q;
        cur_node -> qcy += prt[cur_node -> part_idx].y * prt[cur_node -> part_idx].q;
        cur_node -> mcx += prt[cur_node -> part_idx].x * prt[cur_node -> part_idx].m;
        cur_node -> mcy += prt[cur_node -> part_idx].y * prt[cur_node -> part_idx].m;
        //std::cout << "has part: d " << depth << "\n";
    }
    cur_node -> qcx /= cur_node -> qs;
    cur_node -> qcy /= cur_node -> qs;
    cur_node -> mcx /= cur_node -> ms;
    cur_node -> mcy /= cur_node -> ms;
    //std::cout << "add: updated\n";
}

void quad_tree::calc_one(int idx, qt_node *cur_node, int depth){
    double cur_dist = dist(idx, cur_node);
    //std::cout << idx << " " << (void *) cur_node << " " << depth << std::endl;
    if ((cur_dist < 0.0) || (cur_node -> part_idx == -1))
        return;
    double dx, dy, r, r2, r3, E, F;
    if (((depth > 15) && (cur_dist > 1.0))
        || (cur_dist > dist_m)){
        dx = cur_node -> qcx - prt[idx].x;
        dy = cur_node -> qcy - prt[idx].y;
        r2 = sqr(dx) + sqr(dy);
        r2 += sqr(re);
        r = sqrt(r2);
        r3 = r * r2;
        E = ue ? (- k_el * cur_node -> qs * prt[idx].q / r3) : 0.0;
        cur_ax += E * dx;
        cur_ay += E * dy;

        dx = cur_node -> mcx - prt[idx].x;
        dy = cur_node -> mcy - prt[idx].y;
        r2 = sqr(dx) + sqr(dy);
        r2 += sqr(re);
        r = sqrt(r2);
        r3 = r * r2;
        F = ug ? (G * cur_node -> ms * prt[idx].m / r3) : 0.0;
        cur_ax += F * dx;
        cur_ay += F * dy;
        //std::cout << "case dist\n";
        return;
    }
    if (cur_node -> part_idx == idx){
        //std::cout << "case idx\n";
        if (cur_node -> is_leaf)
            return;
        for(int i = 0; i < 4; i ++){
            calc_one(idx, cur_node -> nodes[i], depth + 1);
        }
        return;
    }
    dx = prt[cur_node -> part_idx].x - prt[idx].x;
    dy = prt[cur_node -> part_idx].y - prt[idx].y;
    r2 = sqr(dx) + sqr(dy);
    r2 += sqr(re);
    r = sqrt(r2);
    r3 = r * r2;
    E = ue ? (- k_el * prt[cur_node -> part_idx].q * prt[idx].q / r3) : 0.0;
    cur_ax += E * dx;
    cur_ay += E * dy;

    dx = prt[cur_node -> part_idx].x - prt[idx].x;
    dy = prt[cur_node -> part_idx].y - prt[idx].y;
    r2 = sqr(dx) + sqr(dy);
    r2 += sqr(re);
    r = sqrt(r2);
    r3 = r * r2;
    F = ug ? (G * prt[cur_node -> part_idx].m * prt[idx].m / r3) : 0.0;
    cur_ax += F * dx;
    cur_ay += F * dy;
    if (!(cur_node -> is_leaf))
        for(int i = 0; i < 4; i ++){
            calc_one(idx, cur_node -> nodes[i], depth + 1);
        }
    //std::cout << "case other\n";
    return;
}

void quad_tree::clear(qt_node *cur_node){
    cur_node -> is_leaf = true;
    for(int i = 0; i < 4; i ++){
        if((cur_node -> nodes[i]) -> is_leaf)
            delete (cur_node -> nodes[i]);
        else
            clear(cur_node -> nodes[i]);
    }
}


EM_engine::EM_engine(int mode)
{
    ugrav = (bool)(mode & USE_GRAV);
    uel = (bool)(mode & USE_EL);
    uspr = (bool)(mode & USE_SPRNG);
    urk = (bool)(mode & USE_RK);
    uqt = (bool)(mode & USE_QT);
    dmp = (bool)(mode & USE_DUMP);
    void (EM_engine::*integr_method)(void) = &EM_engine::integr_euler;
    if (mode & USE_RK)
        void (EM_engine::*integr_method)(void) = &EM_engine::integr_rk;
    void (EM_engine::*calc_method)(void) = &EM_engine::calc_accel;
    if (mode & USE_QT)
        void (EM_engine::*calc_method)(void) = &EM_engine::calc_accel_qt;
    cycles = 0;
}

EM_engine::~EM_engine()
{

}

void EM_engine::calc_spr(){
    if (!uspr)
        return;
    for(int si = 0, ss = springs.size(); si < ss; si ++){
        int pi = springs[si].p1;
        int pj = springs[si].p2;
        if (particles[pi].is_stat)
            continue;
        if (pi == pj)
            continue;
        double dx = particles[pj].x - particles[pi].x;
        double dy = particles[pj].y - particles[pi].y;
        double r2 = sqr(dx) + sqr(dy);
        if (r2 < 1e-30)
            continue;
        double r = sqrt(r2);
        double S = springs[si].k * (r - springs[si].l0);
        if (!particles[pi].is_stat){
            particles[pi].ax += S * (dx / r) / particles[pi].m;
            particles[pi].ay += S * (dy / r) / particles[pi].m;
        }
        if (!particles[pj].is_stat){
            particles[pj].ax += - S * (dx / r) / particles[pj].m;
            particles[pj].ay += - S * (dy / r) / particles[pj].m;
        }
    }
}

void EM_engine::calc_accel(){
    for(int pi = 0, ps = particles.size(); pi < ps; pi ++){
        particles[pi].ax = 0.0;
        particles[pi].ay = 0.0;
        if (particles[pi].is_stat)
            continue;
        for(int pj = 0; pj < ps; pj ++){
            if (pi == pj)
                continue;
            double dx = particles[pj].x - particles[pi].x;
            double dy = particles[pj].y - particles[pi].y;
            double r2 = sqr(dx) + sqr(dy);
            r2 += sqr(re);
            double r = sqrt(r2);
            double r3 = r * r2;
            double E = uel ? (- k_el * particles[pi].q * particles[pj].q / r3) : 0.0;
            double F = ugrav ? (G * particles[pi].m * particles[pj].m / r3) : 0.0;
            particles[pi].ax += (E + F) * (dx);
            particles[pi].ay += (E + F) * (dy);
        }
    }
    for(int pi = 0, ps = particles.size(); pi < ps; pi ++){
        particles[pi].ax /= particles[pi].m;
        particles[pi].ay /= particles[pi].m;
    }
    calc_spr();
}

void EM_engine::calc_accel_qt(){
    qt = new quad_tree(particles, uel, ugrav);
    qt -> get_res(particles);
    calc_spr();
    delete qt;
}

void EM_engine::calc_next(){
    cycles ++;
    urk ? integr_rk() : integr_euler();
    if (dmp)
        dump();
    //(this ->*integr_method)();
}

int EM_engine::add_part(particle *prt){
    if (prt -> m == 0.0)
        return -1;
    particles.push_back(*prt);
    return 0;
}

int EM_engine::del_part(int idx){
    for(int si = springs.size() - 1; si >= 0; si --){
        if ((springs[si].p1 == idx) || (springs[si].p2 == idx))
            springs.erase(springs.begin() + si);
    }
    if (idx != ((long long)particles.size() - 1ll))
        for(int si = springs.size() - 1; si >= 0; si --){
            if (springs[si].p1 > idx)
                springs[si].p1 --;
            if (springs[si].p2 > idx)
                springs[si].p2 --;
        }
    particles.erase(particles.begin() + idx);
    return 0;
}

int EM_engine::add_spring(spring *spr){
    if ((spr -> k == 0.0) || (spr -> l0 == 0.0) ||
        (spr -> p1 < 0) || (spr -> p1 >= (long long) particles.size()) ||
        (spr -> p2 < 0) || (spr -> p2 >= (long long) particles.size()))
        return -1;
    springs.push_back(*spr);
    return 0;
}

int EM_engine::del_spring(int p1, int p2){
    for(int si = springs.size() - 1; si >= 0; si --)
        if (((springs[si].p1 == p1) && (springs[si].p2 == p2)) ||
            ((springs[si].p1 == p2) && (springs[si].p2 == p1)))
            springs.erase(springs.begin() + si);
    return 0;
}

void EM_engine::integr_euler(){
    //(this ->*calc_method)();
    if (uqt)
        calc_accel_qt();
    else
        calc_accel();
    for(int pi = 0, ps = particles.size(); pi < ps; pi ++){
        particles[pi].x += particles[pi].vx * dt;
        particles[pi].y += particles[pi].vy * dt;
        particles[pi].vx += particles[pi].ax * dt;
        particles[pi].vy += particles[pi].ay * dt;
    }
}

void EM_engine::integr_rk(){
    for(int pi = 0, ps = particles.size(); pi < ps; pi ++){
        particles[pi].cur_x = particles[pi].x;
        particles[pi].cur_y = particles[pi].y;
        particles[pi].cur_vx = particles[pi].vx;
        particles[pi].cur_vy = particles[pi].vy;
    }
    //(this ->*calc_method)();
    if (uqt)
        calc_accel_qt();
    else
        calc_accel();
    for(int pi = 0, ps = particles.size(); pi < ps; pi ++){
        particles[pi].k1_x = particles[pi].vx * dt;
        particles[pi].k1_y = particles[pi].vy * dt;
        particles[pi].k1_vx = particles[pi].ax * dt;
        particles[pi].k1_vy = particles[pi].ay * dt;
    }
    for(int pi = 0, ps = particles.size(); pi < ps; pi ++){
        particles[pi].x = particles[pi].cur_x + particles[pi].k1_x / 2.0;
        particles[pi].y = particles[pi].cur_y + particles[pi].k1_y / 2.0;
        particles[pi].vx = particles[pi].cur_vx + particles[pi].k1_vx / 2.0;
        particles[pi].vy = particles[pi].cur_vy + particles[pi].k1_vy / 2.0;
    }
    //(this ->*calc_method)();
    if (uqt)
        calc_accel_qt();
    else
        calc_accel();
    for(int pi = 0, ps = particles.size(); pi < ps; pi ++){
        particles[pi].k2_x = particles[pi].vx * dt;
        particles[pi].k2_y = particles[pi].vy * dt;
        particles[pi].k2_vx = particles[pi].ax * dt;
        particles[pi].k2_vy = particles[pi].ay * dt;
    }
    for(int pi = 0, ps = particles.size(); pi < ps; pi ++){
        particles[pi].x = particles[pi].cur_x + particles[pi].k2_x / 2.0;
        particles[pi].y = particles[pi].cur_y + particles[pi].k2_y / 2.0;
        particles[pi].vx = particles[pi].cur_vx + particles[pi].k2_vx / 2.0;
        particles[pi].vy = particles[pi].cur_vy + particles[pi].k2_vy / 2.0;
    }
    //(this ->*calc_method)();
    if (uqt)
        calc_accel_qt();
    else
        calc_accel();
    for(int pi = 0, ps = particles.size(); pi < ps; pi ++){
        particles[pi].k3_x = particles[pi].vx * dt;
        particles[pi].k3_y = particles[pi].vy * dt;
        particles[pi].k3_vx = particles[pi].ax * dt;
        particles[pi].k3_vy = particles[pi].ay * dt;
    }
    for(int pi = 0, ps = particles.size(); pi < ps; pi ++){
        particles[pi].x = particles[pi].cur_x + particles[pi].k3_x;
        particles[pi].y = particles[pi].cur_y + particles[pi].k3_y;
        particles[pi].vx = particles[pi].cur_vx + particles[pi].k3_vx;
        particles[pi].vy = particles[pi].cur_vy + particles[pi].k3_vy;
    }
    //(this ->*calc_method)();
    if (uqt)
        calc_accel_qt();
    else
        calc_accel();
    for(int pi = 0, ps = particles.size(); pi < ps; pi ++){
        particles[pi].k4_x = particles[pi].vx * dt;
        particles[pi].k4_y = particles[pi].vy * dt;
        particles[pi].k4_vx = particles[pi].ax * dt;
        particles[pi].k4_vy = particles[pi].ay * dt;
    }
    for(int pi = 0, ps = particles.size(); pi < ps; pi ++){
        particles[pi].x = particles[pi].cur_x +
            (particles[pi].k1_x + 2.0 * particles[pi].k2_x + 2.0 * particles[pi].k3_x + particles[pi].k4_x) / 6.0;
        particles[pi].y = particles[pi].cur_y +
            (particles[pi].k1_y + 2.0 * particles[pi].k2_y + 2.0 * particles[pi].k3_y + particles[pi].k4_y) / 6.0;
        particles[pi].vx = particles[pi].cur_vx +
            (particles[pi].k1_vx + 2.0 * particles[pi].k2_vx + 2.0 * particles[pi].k3_vx + particles[pi].k4_vx) / 6.0;
        particles[pi].vy = particles[pi].cur_vy +
            (particles[pi].k1_vy + 2.0 * particles[pi].k2_vy + 2.0 * particles[pi].k3_vy + particles[pi].k4_vy) / 6.0;
    }
}

void EM_engine::dump(){
    std::cout << "cycle #" << cycles << std::endl;
    for(int pi = 0, ps = particles.size(); pi < ps; pi ++)
        std::cout << "p" << pi << ": x = " << particles[pi].x << " y = " << particles[pi].y << std::endl;
    std::cout << std::endl;
}
