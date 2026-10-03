#ifndef SCENE_H
#define SCENE_H

#include "Fluid.h"
#include "FluidVisualizer.h"
#include <vector>

namespace asa
{
class Scene
{
public:
    enum { TEST_ADVECTION, SMOKE };

public:
    // settings
    static int testcase;
    static bool pauseFlag;

    static uint nCellsX;
    static uint nCellsY;
    static float step;
    static float kDensity;
    static float kGravity;
    static float kViscosity;
    static bool kEnableWind;

public:
    Scene();
    ~Scene();

    const Fluid *getFluid() const { return fluid; }
    Fluid *getFluid() { return fluid; }

    const FluidVisualizer *getFluidViz() const { return fluidViz; }
    FluidVisualizer *getFluidViz() { return fluidViz; }

    // initialization
    void init();
    void init(int argc, char *argv[]);
    void initAnimation();
    void printSettings();

    // Update
    void pause();
    void update();
    void animate();

    // Display
    void display();

private:
    Fluid *fluid;
    FluidVisualizer *fluidViz;
};
};  // namespace asa

#endif
