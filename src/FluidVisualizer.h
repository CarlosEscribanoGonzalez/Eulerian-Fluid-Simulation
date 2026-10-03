#ifndef FLUID_VISUALIZER_2_H
#define FLUID_VISUALIZER_2_H

#include "Fluid.h"

#include <GL/glut.h>
#include <iostream>

namespace asa
{
class FluidVisualizer
{
public:
    FluidVisualizer(const Fluid &fluid);
    ~FluidVisualizer();

    bool getIsVisibleGrid() const { return isVisibleGrid; }

    void init();

    void draw();

    void toggleVisibleGrid();

private:
    void drawGrid();
    void drawInkField();
    void drawVelocityField();
    void drawPressureField();

    void drawBbox(const AABox2 &bbox);

private:
    const Fluid &fluid;

    bool isVisibleGrid;

    GLuint texture[3];
};
};  // namespace asa

#endif
