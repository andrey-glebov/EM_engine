#ifdef __cplusplus
    #include <cstdlib>
#else
    #include <stdlib.h>
#endif
#ifdef __APPLE__
#include <SDL/SDL.h>
#else
#include <SDL.h>
#endif

#include <ctime>

#include "include/EM_engine.h"

#define resize_coef 100

double rnd1(double lim){
    double r = lim * 1.0 * (rand() / (double)RAND_MAX - 0.5);
    //std::cout << "r1: " << r << std::endl;
    return r;
}
double rnd2(double lim){
    double r = lim * 1.0 * ((rand() + 1ll) / (double)RAND_MAX);
    //std::cout << "r2: " << r << std::endl;
    return r;
}
double rnd3(double lim1, double lim2){
    double r = lim1 + (lim2 - lim1) * 1.0 * ((rand() + 1ll) / (double)RAND_MAX);
    //std::cout << "r2: " << r << std::endl;
    return r;
}

EM_engine *EME = new EM_engine();
particle prt0;

void put_pixel(SDL_Surface* surface, int x, int y, Uint32 pixel);

int main ( int argc, char** argv )
{
    // initialize SDL video
    if ( SDL_Init( SDL_INIT_VIDEO ) < 0 )
    {
        printf( "Unable to init SDL: %s\n", SDL_GetError() );
        return 1;
    }

    // make sure SDL cleans up before exit
    atexit(SDL_Quit);

    // create a new window
    SDL_Surface* screen = SDL_SetVideoMode(960, 720, 32,
                                           SDL_HWSURFACE|SDL_DOUBLEBUF);
    if ( !screen )
    {
        printf("Unable to set 960x720 video: %s\n", SDL_GetError());
        return 1;
    }

/*
    // load an image
    SDL_Surface* bmp = SDL_LoadBMP("cb.bmp");
    if (!bmp)
    {
        printf("Unable to load bitmap: %s\n", SDL_GetError());
        return 1;
    }

    // centre the bitmap on screen
    SDL_Rect dstrect;
    dstrect.x = (screen->w - bmp->w) / 2;
    dstrect.y = (screen->h - bmp->h) / 2;
*/
    // program main loop

    srand(time(0) ^ clock());

    for(int i = 0; i < 3000; i ++){
        prt0.q = 0.0;
        prt0.m = rnd3(5e8, 1e9);
        prt0.x = rnd3(2.0, 5.0);
        prt0.y = rnd3(2.0, 5.0);
        prt0.vx = 0.0;
        prt0.vy = 0.0;
        prt0.ax = 0.0;
        prt0.ay = 0.0;
        EME -> add_part(&prt0);
    }

    bool done = false;
    while (!done)
    {
        // message processing loop
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            // check for messages
            switch (event.type)
            {
                // exit if the window is closed
            case SDL_QUIT:
                done = true;
                break;

                // check for keypresses
            case SDL_KEYDOWN:
                {
                    // exit if ESCAPE is pressed
                    if (event.key.keysym.sym == SDLK_ESCAPE)
                        done = true;
                    break;
                }
            } // end switch
        } // end of message processing

        // DRAWING STARTS HERE

        // clear screen
        SDL_FillRect(screen, 0, SDL_MapRGB(screen->format, 0, 0, 0));
/*
        // draw bitmap
        SDL_BlitSurface(bmp, 0, screen, &dstrect);
*/
        SDL_LockSurface(screen);

        for(int pi = 0, ps = EME -> particles.size(); pi < ps; pi ++){
            put_pixel(screen,
                      EME -> particles[pi].x * resize_coef, EME -> particles[pi].y * resize_coef,
                      0x00000000
                     );
        }

        EME -> calc_new();

        for(int pi = 0, ps = EME -> particles.size(); pi < ps; pi ++){
            put_pixel(screen,
                      EME -> particles[pi].x * resize_coef, EME -> particles[pi].y * resize_coef,
                      ((EME -> particles[pi].q > 0) ? 0x00ff0000 : 0x0000ff00)
                     );
        }

        SDL_UnlockSurface(screen);

        // DRAWING ENDS HERE

        // finally, update the screen :)
        SDL_Flip(screen);
    } // end main loop

    // free loaded bitmap
    //SDL_FreeSurface(bmp);

    // all is well ;)
    printf("Exited cleanly\n");
    return 0;
}

void put_pixel(SDL_Surface* surface, int x, int y, Uint32 pixel)
{
    if ((x < 0) || (y < 0) || (x >= surface->w) || (y >= surface->h))
        return;

    int bpp = surface->format->BytesPerPixel;
    /* Here p is the address to the pixel we want to set */
    Uint8 *p = (Uint8 *)surface->pixels + y * surface->pitch + x * bpp;

    switch(bpp) {
    case 1:
        *p = pixel;
        break;

    case 2:
        *(Uint16 *)p = pixel;
        break;

    case 3:
        if(SDL_BYTEORDER == SDL_BIG_ENDIAN) {
            p[0] = (pixel >> 16) & 0xff;
            p[1] = (pixel >> 8) & 0xff;
            p[2] = pixel & 0xff;
        } else {
            p[0] = pixel & 0xff;
            p[1] = (pixel >> 8) & 0xff;
            p[2] = (pixel >> 16) & 0xff;
        }
        break;

    case 4:
        *(Uint32 *)p = pixel;
        break;
    }
}
