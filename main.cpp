// #include "PVector.hpp"
#include "bodies.hpp"
#include "colorBufferLUT.hpp" // for generating pretty colors

#include <chrono>

extern uint8_t colorBufferLUT[];

// #include <iostream> // included in hpp file

// SDL stuff
#include <SDL2/SDL.h> 
#include <SDL2/SDL_image.h> 
#include <SDL2/SDL_video.h>
#include <SDL2/SDL_timer.h> 

#define WINDOW_WIDTH    (640)
#define WINDOW_HEIGHT   (480)

#define SPEED    (300)
#define BUFFER   (10)

// #define STEP_DEBUG

SDL_Point toSDL(Rectangle * source,uint32_t n){
    SDL_Point out;
    out.x = source->painter.x[n];
    out.y = source->painter.y[n];
    return out;
}

// for now, overloaded.. eventually want to make polymorphism! inheritence!
    // shape class
SDL_Point toSDL(Ball * source,uint32_t n){
    SDL_Point out;
    out.x = source->painter.x[n];
    out.y = source->painter.y[n];
    return out;
}

int main(){
    Vector2f initPos(20,25);
    Vector2f initVel(0,0);
    Vector2f initAcc(0,0);
    // Vector3f initAttitude((3*pi)/2,((2*pi)/2)*dt,(pi/100)*dt);
    Vector3f initAttitude(pi/8,((2*pi)/1000)*dt,0);

    Rectangle boxBoi(20,4,initPos,initVel,initAcc,initAttitude); // need to construct this first to get accurate pos
	Ball roundBoi(5,boxBoi.pos,initVel,initAcc); // 20 meters
	// Ball copy(4,20,20);

    if (SDL_Init(SDL_INIT_VIDEO|SDL_INIT_TIMER) != 0) { 
        printf("error initializing SDL: %s\n", SDL_GetError()); 
    } 
    SDL_Window* win = SDL_CreateWindow("Window Title",// title of window
                                       SDL_WINDOWPOS_CENTERED, // x position
                                       SDL_WINDOWPOS_CENTERED, // y position
                                       WINDOW_WIDTH, // width
                                       WINDOW_HEIGHT, // height
                                       0); // flags
    if(!win){
        printf("error creating window: %s\n",SDL_GetError());
        SDL_Quit();
        return 1;
    }
    // ^ returns null on failure
    uint32_t render_flags = SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC; // can get more documentation details
    // accelerated uses graphics hardware!
    // vsync prevents screen tear
    SDL_Renderer* rend = SDL_CreateRenderer(win, // SDL_Window
    										-1, // index
    										render_flags); // flags
    // returns a rendering context
    // Goes along with the window, abstract object in charge of doing all of the 'drying' operations
    // corresponds to a program loaded into the graphics hardware
    if(!rend){
    	printf("Error creating renderer: %s\n",SDL_GetError());
    	SDL_DestroyWindow(win);
    	SDL_Quit();
    	return 1;
    }
    // Instead of using images, I'm going to draw some rectangles!
    // SDL interprets each pixel as a 32-bit number, so our masks depend on the endianness of the machine
    uint32_t rmask, gmask, bmask, amask;
#if(SDL_BYTEORDER == SDL_BIG_ENDIAN)
    rmask = 0xff000000;
    gmask = 0x00ff0000;
    bmask = 0x0000ff00;
    amask = 0x000000ff;
#else
    rmask = 0x000000ff;
    gmask = 0x0000ff00;
    bmask = 0x00ff0000;
    amask = 0xff000000; // why not 0x00ff0000???
#endif
    // OKAY: so surface width and height was fucked up...
    // BUT: why did it 'stretch' to the window scale? 300/50 --> 640/480??
    SDL_Surface * surface = SDL_CreateRGBSurface(0,     // flags are 'unused' and should be set to 0
                                                 WINDOW_WIDTH,// 300,//width
                                                 WINDOW_HEIGHT,// 50,// height
                                                 32, // Depth of surface, in bits (4 or 8 bits)
                                                 rmask,
                                                 gmask,
                                                 bmask,
                                                 amask);

    // SDL_Surface* surface = IMG_Load("resources/hello.png"); // only function used from SDL_image library
    // surface struct represents image data in memory
    if(!surface){
    	printf("Error creating surface: %s\n",SDL_GetError());
    	SDL_DestroyRenderer(rend);
    	SDL_DestroyWindow(win);
    	SDL_Quit();
    	return 1;	
    }
    // load image into graphics hardware's memory
    SDL_Texture* tex = SDL_CreateTextureFromSurface(rend,surface);
    // texture is in VRAM memory in graphics card!
    SDL_FreeSurface(surface);
    if(!tex){
    	printf("Error creating texture: %s\n",SDL_GetError());
    	SDL_DestroyRenderer(rend);
    	SDL_DestroyWindow(win);
    	SDL_Quit();
    	return 1;
    }
    // going to just spin the circle
    uint8_t close_requested = 0, start_running = 0;
    bool step = false;
    // int mouse_x, mouse_y;
    while(!close_requested){
        // process events
        SDL_Event event;
        // only gets executed in first run
        if(!start_running){
        	while(SDL_PollEvent(&event)){
        		if(event.key.keysym.scancode == SDL_SCANCODE_ESCAPE){
        			start_running = 1;
        		}
        	}
        	continue;
        }
        while(SDL_PollEvent(&event)){
        	if(event.key.keysym.scancode == SDL_SCANCODE_Q){
        		close_requested = 1;
        	} else if(event.key.keysym.scancode == SDL_SCANCODE_R){
        		// roundBoi = copy.copy();
                ;
        	}
        }
        while(step){
        	while(SDL_PollEvent(&event)){
        		if(event.key.keysym.scancode == SDL_SCANCODE_S){
        			step = false;
        		}
        	}
        }
        auto start = std::chrono::high_resolution_clock::now();
	    
        boxBoi.move();
        boxBoi.update();
        auto boxStop = std::chrono::high_resolution_clock::now();
        auto boxDur = std::chrono::duration_cast<std::chrono::microseconds>(boxStop - start);
        // std::cout << boxDur.count() << std::endl;

        // SDL_Point paint[roundBoi.painter.x.size()];
        SDL_Point paintB[boxBoi.painter.x.size()];
        auto start0 = std::chrono::high_resolution_clock::now();
        for(uint32_t i=0;i<boxBoi.painter.x.size();i++){
            paintB[i] = toSDL(&boxBoi,i);
        }

        // clear the window
        SDL_SetRenderDrawColor(rend,0,0,0,0);
        SDL_RenderClear(rend);

        // draw the image/sprite to the window
        // these will be represent the elements of the LUT
        static uint8_t R=40-1,G=80-1,B=120-1;
        R = (R < 119)? R+1 : 0;
        G = (G < 119)? G+1 : 0;
        B = (B < 119)? B+1 : 0;


        SDL_SetRenderDrawColor(rend,colorBufferLUT[R],colorBufferLUT[G],colorBufferLUT[B],0); // orange


        // draw the pixels
        SDL_RenderDrawPoints(rend,
                            paintB,
                            // ,roundBoi.painter.x.size());
                            boxBoi.painter.x.size());
        auto stop0 = std::chrono::high_resolution_clock::now();
        auto dur0 = std::chrono::duration_cast<std::chrono::microseconds>(stop0 - start0);
        // std::cout << dur0.count() << std::endl;
        auto roundStart = std::chrono::high_resolution_clock::now();
        roundBoi.move();
        roundBoi.update();
        auto roundEnd = std::chrono::high_resolution_clock::now();
        auto roundDur = std::chrono::duration_cast<std::chrono::microseconds>(roundEnd - roundStart);
        // std::cout << roundDur.count() << std::endl;
        SDL_Point paintR[roundBoi.painter.x.size()];
        auto start1 = std::chrono::high_resolution_clock::now();
        for(uint32_t i=0;i<roundBoi.painter.x.size();i++){
            paintR[i] = toSDL(&roundBoi,i);
        }

        SDL_SetRenderDrawColor(rend,0xff,0xfF,0xff,0); // aqua

        SDL_RenderDrawPoints(rend,
                            paintR,
                            roundBoi.painter.x.size());

        SDL_RenderPresent(rend); // swap double buffers
        auto stop1 = std::chrono::high_resolution_clock::now();
        auto dur1 = std::chrono::duration_cast<std::chrono::microseconds>(stop1 - start1);
        // std::cout << dur1.count() << std::endl;
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        // std::cout << duration.count() << std::endl;
        float miliElapsed = static_cast<float>(duration.count())/1000;
        static float frameTime = 1000*dt;
        int32_t delay = roundHelper(frameTime - miliElapsed);
        delay = (delay < 0)? 0 : delay;
        if(!delay){
            std::cout << miliElapsed << std::endl;
        }
	    
        if(delay){
            SDL_Delay(delay); // add 0.5 because casting always rounds down
        }
        // actually still need to calculate the time it took to do calculations from top of loop to this delay part
#ifdef STEP_DEBUG
        step = true;
#endif // STEP_DEBUG
    }

    SDL_Delay(5000);

    // SDL_GetDisplayBounds(0,&rect);

    /* END?? */

    // clean up resources before exit
    SDL_DestroyTexture(tex);
    SDL_DestroyRenderer(rend);
    SDL_DestroyWindow(win);
    SDL_Quit(); // destroy window, and cleanup library
    // while (1) 
        // ; 

    // clean up!
  
    return 0; 
} 
