## Overview
Implementation of a 2D Eulerian fluid simulator that solves the incompressible **Navier-Stokes equations** on a staggered (MAC) grid, written in C++. Each time step handles the terms of the equations one at a time: advection, external forces, viscosity and a pressure projection that makes the fluid incompressible. Colored ink is advected along the fluid to visualize the flow. The domain is fixed and solid on all four sides.

## Features
* **Ink emission:** several colored emitters (RGB) inject ink and velocity into the fluid, producing plumes of smoke
* **Semi-Lagrangian advection:** ink (stored at cell centers) and the two velocity components (stored at cell faces) are advected by tracing back along the velocity field and interpolating bilinearly, which keeps the simulation stable at large time steps
* **Volume forces:**
  * Gravity
  * Configurable wind regions that push the fluid horizontally or vertically
* **Viscosity:** diffusion of the velocity field through the Laplacian, scaled by the dynamic viscosity and the fluid density
* **Pressure projection:**
  * Solid boundary conditions at the four walls
  * Divergence of the velocity field computed on the grid
  * Pressure Poisson system solved with a preconditioned conjugate gradient solver
  * Pressure gradient subtracted from the velocities to make the fluid incompressible

## Configurable parameters
Gravity, fluid density, viscosity, time step, wind and the grid dimensions (number of cells per axis) can be changed in `Scene.cpp`. Trying different resolutions shows how the cell size affects both the look of the simulation and the computation time.

## Controls
* **S:** pause / resume the simulation
* **G:** show the underlying grid
* **ESC:** exit

## Authorship
I only developed the simulation steps in `Simulation.cpp`: emission, advection, volume forces, viscosity and pressure projection. The rest of the project (simulation framework, grid and array classes, sparse matrix and PCG solver, rendering and build files) was provided by the professor and is included only so the project can run.

## Usage
* Run `gen_prj.cmd`
* Enter the `build` folder and open the generated `.sln` in Visual Studio
* Build and run the project

## Requirements
* Visual Studio
* CMake
