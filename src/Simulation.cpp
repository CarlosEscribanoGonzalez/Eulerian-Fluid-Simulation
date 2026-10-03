#include "Scene.h"

#include "Numeric/PCGSolver.h"
namespace asa
{
#pragma region STRUCTS & ENUMS
enum Axis { Horizontal, Vertical };

struct Emissor {
    Vector2 domainX;
    Vector2 domainY;
    Vector3 ink;
    float speedX;
    float speedY;
};

struct Wind {
    Vector2 domainX;
    Vector2 domainY;
    float speedX;
    float speedY;
};
#pragma endregion

#pragma region FUNCIONES AUXILIARES
static bool isInDomain(Vector2 pos, Vector2 domainX, Vector2 domainY)
{
    bool in_x = pos.x >= domainX.x && pos.x <= domainX.y;
    bool in_y = pos.y >= domainY.x && pos.y <= domainY.y;
    return in_x && in_y;
}

static void checkEmission(Index2 idx, const Emissor &emissor, const Grid2 &grid, Array2<Vector3> &ink,
        Array2<float> &velX, Array2<float> &velY)
{
    // Tinta:
    if (isInDomain(grid.getCellPos(idx), emissor.domainX, emissor.domainY))
        ink[idx] = emissor.ink;
    // Velocidades horizontales:
    if (isInDomain(grid.getFacePosX(idx), emissor.domainX, emissor.domainY))
        velX[idx] = emissor.speedX;
    Index2 nextIdx_x{idx.x + 1, idx.y};
    if (isInDomain(grid.getFacePosX(nextIdx_x), emissor.domainX, emissor.domainY))
        velX[nextIdx_x] = emissor.speedX;
    // Velocidades verticales:
    if (isInDomain(grid.getFacePosY(idx), emissor.domainX, emissor.domainY))
        velY[idx] = emissor.speedY;
    Index2 nextIdx_y{idx.x, idx.y + 1};
    if (isInDomain(grid.getFacePosY(nextIdx_y), emissor.domainX, emissor.domainY))
        velY[nextIdx_y] = emissor.speedY;
}

static void checkWind(Index2 idx, const Wind& wind, const Grid2 &grid, 
    Array2<float> &velX, Array2<float> &velY, float dt, bool override = false)
{
    // Velocidades horizontales:
    if (isInDomain(grid.getFacePosX(idx), wind.domainX, wind.domainY))
        velX[idx] = override ? wind.speedX : velX[idx] + wind.speedX * dt;
    Index2 nextIdx_x{idx.x + 1, idx.y};
    if (isInDomain(grid.getFacePosX(nextIdx_x), wind.domainX, wind.domainY))
        velX[nextIdx_x] = override ? wind.speedX : velX[nextIdx_x] + wind.speedX * dt;
    // Velocidades verticales:
    if (isInDomain(grid.getFacePosY(idx), wind.domainX, wind.domainY))
        velY[idx] = override ? wind.speedY : velY[idx] + wind.speedY * dt;
    Index2 nextIdx_y{idx.x, idx.y + 1};
    if (isInDomain(grid.getFacePosY(nextIdx_y), wind.domainX, wind.domainY))
        velY[nextIdx_y] = override ? wind.speedY : velY[nextIdx_y] + wind.speedY * dt;
}

static Index2 clampedIndex(int x, int y, Index2 bounds)
{
    x = clamp(x, 0, bounds.x - 1);
    y = clamp(y, 0, bounds.y - 1);
    return {(uint)x, (uint)y};
}

static void clampPosition(Vector2& pos, const Grid2& grid)
{
    pos.x = clamp(pos.x, grid.getDomain().minPosition.x, grid.getDomain().maxPosition.x);
    pos.y = clamp(pos.y, grid.getDomain().minPosition.y, grid.getDomain().maxPosition.y);
}

template<class T>
static T bilerp_center(Vector2 &pos, const Grid2 &grid, const Array2<T> &values) {
    //Clampeo de posición a una dentro del dominio
    clampPosition(pos, grid);
    //Obtención de coordenadas en relación al centro de las celdas
    Vector2 coord = grid.getCellIndex(pos);
    float floor_x = max(floor(coord.x), 0.0f); //Coge la celda cuyo centro está a la izquierda de pos
    float floor_y = max(floor(coord.y), 0.0f); //Coge la celda cuyo centro está debajo de pos
    Index2 idx{(uint)floor_x, (uint)floor_y};
    uint nextIdx_x = min(idx.x + 1, grid.getSize().x - 1); //Celda derecha
    uint nextIdx_y = min(idx.y + 1, grid.getSize().y - 1); //Celda superior
    float tx = clamp(coord.x - floor_x, 0.0f, 1.0f);
    float ty = clamp(coord.y - floor_y, 0.0f, 1.0f);
    return bilerp(values[idx], values[{nextIdx_x, idx.y}], 
        values[{idx.x, nextIdx_y}], values[{nextIdx_x, nextIdx_y}], tx, ty);
}

template <class T>
static T bilerp_face(Vector2 &pos, const Grid2 &grid, const Array2<T> &values, Axis axis)
{
    clampPosition(pos, grid);
    Vector2 coord = axis == Horizontal ? grid.getFaceIndexX(pos) : grid.getFaceIndexY(pos);
    float floor_x = max(floor(coord.x), 0.0f);
    float floor_y = max(floor(coord.y), 0.0f);
    Index2 idx{(uint)floor_x, (uint)floor_y};
    Index2 sizeFaces = axis == Horizontal ? grid.getSizeFacesX() : grid.getSizeFacesY();
    uint nextIdx_x = min(idx.x + 1, sizeFaces.x - 1);
    uint nextIdx_y = min(idx.y + 1, sizeFaces.y - 1);
    float tx = clamp(coord.x - floor_x, 0.0f, 1.0f);
    float ty = clamp(coord.y - floor_y, 0.0f, 1.0f);
    return bilerp(values[idx], values[{nextIdx_x, idx.y}], 
        values[{idx.x, nextIdx_y}], values[{nextIdx_x, nextIdx_y}], tx, ty);
}
#pragma endregion

SparseMatrix<float> A(1, 5);
bool isAInitialized = false;
bool enableWind = false;

void Fluid::fluidAdvection(const float dt)
{
    // TINTA:
    {
        Array2<Vector3> inkCopy = inkRGB;
        for (uint i = 0; i < grid.getSize().x; i++) {
            for (uint j = 0; j < grid.getSize().y; j++) {
                Index2 idx{i, j};
                // Obtención de velocidad y posición:
                Vector2 currentPos = grid.getCellPos(idx);
                float currentSpeed_x = 0.5 * (velocityX[idx] + velocityX[{i + 1, j}]);
                float currentSpeed_y = 0.5 * (velocityY[idx] + velocityY[{i, j + 1}]);
                Vector2 currentVel{currentSpeed_x, currentSpeed_y};
                // Cálculo de posición previa y actualización:
                Vector2 prevPos = currentPos - dt * currentVel;
                inkRGB[idx] = bilerp_center(prevPos, grid, inkCopy);
            }
        }
    }
    // VELOCIDAD:
    {
        Array2<float> velXCopy = velocityX;
        Array2<float> velYCopy = velocityY;
        // Componentes u:
        for (uint i = 0; i < grid.getSizeFacesX().x; i++) {
            for (uint j = 0; j < grid.getSizeFacesX().y; j++) {
                Index2 idx{i, j};
                // Obtención de velocidad y posición:
                Vector2 currentPos = grid.getFacePosX(idx);
                float u = velXCopy[idx];
                float v = bilerp_face(currentPos, grid, velYCopy, Vertical);
                Vector2 currentVel{u, v};
                // Cálculo de posición previa y actualización:
                Vector2 prevPos = currentPos - dt * currentVel;
                velocityX[idx] = bilerp_face(prevPos, grid, velXCopy, Horizontal);
            }
        }
        // Componentes v:
        for (uint i = 0; i < grid.getSizeFacesY().x; i++) {
            for (uint j = 0; j < grid.getSizeFacesY().y; j++) {
                Index2 idx{i, j};
                // Obtención de velocidad y posición:
                Vector2 currentPos = grid.getFacePosY(idx);
                float u = bilerp_face(currentPos, grid, velXCopy, Horizontal);
                float v = velYCopy[idx];
                Vector2 currentVel{u, v};
                // Cálculo de posición previa y actualización:
                Vector2 prevPos = currentPos - dt * currentVel;
                velocityY[idx] = bilerp_face(prevPos, grid, velYCopy, Vertical);
            }
        }
    }
}

void Fluid::fluidEmission()
{
    if (Scene::testcase >= Scene::SMOKE) { 
        Emissor emissor1{{-0.1, 0.1}, {-1.9, -1.75}, {1.0, 1.0, 0.0}, 0, 8};
        Emissor emissor2{{-0.2, -0.1}, {-1.9, -1.75}, {1.0, 0.0, 1.0}, 0, 8};
        Emissor emissor3{{0.1, 0.2}, {-1.9, -1.75}, {0.0, 1.0, 1.0}, 0, 8};
        for (uint i = 0; i < grid.getSize().x; i++) {
            for (uint j = 0; j < grid.getSize().y; j++) {
                Index2 idx{i, j};
                checkEmission(idx, emissor1, grid, inkRGB, velocityX, velocityY);
                checkEmission(idx, emissor2, grid, inkRGB, velocityX, velocityY);
                checkEmission(idx, emissor3, grid, inkRGB, velocityX, velocityY);
            }
        }
    }
}

void Fluid::fluidVolumeForces(const float dt)
{
    if (Scene::testcase >= Scene::SMOKE) {
        //Gravedad:
        for (uint i = 0; i < grid.getSizeFacesY().x; i++) {
            for (uint j = 0; j < grid.getSizeFacesY().y; j++) {
                velocityY[{i, j}] += Scene::kGravity * dt;
            }
        }
        //Dominios de viento:
        if (!enableWind) return;
        Wind wind1{{-2, 2}, {-0.1, 0.1}, -5, 0};
        Wind wind2{{-0.1, 0.1}, {0, 2}, 0, -1};
        for (uint i = 0; i < grid.getSize().x; i++) {
            for (uint j = 0; j < grid.getSize().y; j++) {
                Index2 idx{i, j};
                checkWind(idx, wind1, grid, velocityX, velocityY, dt);
                checkWind(idx, wind2, grid, velocityX, velocityY, dt);
            }
        }
    }
}

void Fluid::fluidViscosity(const float dt)
{
    if (Scene::testcase >= Scene::SMOKE) {
        float deltaX_2 = grid.getDx().x * grid.getDx().x;
        float deltaY_2 = grid.getDx().y * grid.getDx().y;
        //Componentes u:
        Array2<float> velXCopy = velocityX;
        for (uint i = 0; i < grid.getSizeFacesX().x; i++) {
            for (uint j = 0; j < grid.getSizeFacesX().y; j++) {
                Index2 idx{i, j};
                Index2 idx_prevX = clampedIndex(i - 1, j, grid.getSizeFacesX());
                Index2 idx_nextX = clampedIndex(i + 1, j, grid.getSizeFacesX());
                Index2 idx_prevY = clampedIndex(i, j - 1, grid.getSizeFacesX());
                Index2 idx_nextY = clampedIndex(i, j + 1, grid.getSizeFacesX());
                float viscosity = (velXCopy[idx_nextX] - 2 * velXCopy[idx] + velXCopy[idx_prevX]) / deltaX_2;
                viscosity += (velXCopy[idx_nextY] - 2 * velXCopy[idx] + velXCopy[idx_prevY]) / deltaY_2;
                viscosity *= dt * Scene::kViscosity / Scene::kDensity;
                velocityX[idx] += viscosity;
            }
        }
        //Componentes v:
        Array2<float> velYCopy = velocityY;
        for (uint i = 0; i < grid.getSizeFacesY().x; i++) {
            for (uint j = 0; j < grid.getSizeFacesY().y; j++) {
                Index2 idx{i, j};
                Index2 idx_prevX = clampedIndex(i - 1, j, grid.getSizeFacesY());
                Index2 idx_nextX = clampedIndex(i + 1, j, grid.getSizeFacesY());
                Index2 idx_prevY = clampedIndex(i, j - 1, grid.getSizeFacesY());
                Index2 idx_nextY = clampedIndex(i, j + 1, grid.getSizeFacesY());
                float viscosity = (velYCopy[idx_nextX] - 2 * velYCopy[idx] + velYCopy[idx_prevX]) / deltaX_2;
                viscosity += (velYCopy[idx_nextY] - 2 * velYCopy[idx] + velYCopy[idx_prevY]) / deltaY_2;
                viscosity *= dt * Scene::kViscosity / Scene::kDensity;
                velocityY[idx] += viscosity;
            }
        }
    }
}

void Fluid::fluidPressureProjection(const float dt)
{
    if (Scene::testcase >= Scene::SMOKE) {
        //Poner la velocidad en las fronteras a 0:
        for (uint i = 0; i < grid.getSizeFacesX().y; i++) {
            velocityX[{0, i}] = 0;
            velocityX[{grid.getSizeFacesX().x - 1, i}] = 0;
        }
        for (uint i = 0; i < grid.getSizeFacesY().x; i++) {
            velocityY[{i, 0}] = 0;
            velocityY[{i, grid.getSizeFacesY().y - 1}] = 0;
        }
        // Precisión y tolerancia dependiendo de si queremos velocidad o exactitud
        // float => tolerance_factor = 1e-3, iterations = 200
        // double => tolerance_factor = 1e-6, iterations = 200
        PCGSolver<float> solver;
        solver.set_solver_parameters(1e-3, 200);
        //Llenar RHS:
        std::vector<float> rhs(grid.getSize().x * grid.getSize().y);
        for (uint i = 0; i < grid.getSize().x; i++) {
            for (uint j = 0; j < grid.getSize().y; j++) {
                Index2 idx{i, j};
                Index2 idx_nextX{i + 1, j};
                Index2 idx_nextY{i, j + 1};
                double div = (velocityX[idx_nextX] - velocityX[idx]) / grid.getDx().x;
                div += (velocityY[idx_nextY] - velocityY[idx]) / grid.getDx().y; 
                div *= -Scene::kDensity / dt;
                rhs[i + grid.getSize().x * j] = div;
            }
        }
        //Llenar A:
        if (!isAInitialized) {
            float dx2 = grid.getDx().x * grid.getDx().x;
            float dy2 = grid.getDx().y * grid.getDx().y;
            A.resize(grid.getSize().x * grid.getSize().y);
            for (uint i = 0; i < grid.getSize().x; i++) {
                for (uint j = 0; j < grid.getSize().y; j++) {
                    int rowIdx = i + grid.getSize().x * j;
                    int numNeighbors_x = 0;
                    int numNeighbors_y = 0;
                    if ((int)i - 1 >= 0) {
                        A.set_element(rowIdx, i - 1 + j * grid.getSize().x, -1 / dx2);
                        numNeighbors_x++;
                    }
                    if (i + 1 < grid.getSize().x) {
                        A.set_element(rowIdx, i + 1 + j * grid.getSize().x, -1 / dx2);
                        numNeighbors_x++;
                    }
                    if ((int)j - 1 >= 0) {
                        A.set_element(rowIdx, i + (j - 1) * grid.getSize().x, -1 / dy2);
                        numNeighbors_y++;
                    }
                    if (j + 1 < grid.getSize().y) {
                        A.set_element(rowIdx, i + (j + 1) * grid.getSize().x, -1 / dy2);
                        numNeighbors_y++;
                    }
                    A.set_element(rowIdx, rowIdx, numNeighbors_x / dx2 + numNeighbors_y / dy2);
                }
            }
            isAInitialized = true;
        }
        //Llenar P:
        std::vector<float> P(grid.getSize().x * grid.getSize().y);
        // Solve:
        float residual;
        int iter;
        solver.solve(A, rhs, P, residual, iter);
        // Aplicar las P a la rejilla:
        for (uint j = 0; j < pressure.getSize().y; j++) {
            for (uint i = 0; i < pressure.getSize().x; i++) {
                pressure[{i, j}] = P[i + grid.getSize().x * j];
            }
        }
        float constant = dt / Scene::kDensity;
        for (uint i = 1; i < grid.getSize().x; i++) {
            for (uint j = 0; j < grid.getSize().y; j++) {
                velocityX[{i, j}] -= constant * (pressure[{i, j}] - pressure[{i - 1, j}]) / grid.getDx().x;
            }
        }
        for (uint i = 0; i < grid.getSize().x; i++) {
            for (uint j = 1; j < grid.getSize().y; j++) {
                velocityY[{i, j}] -= constant * (pressure[{i, j}] - pressure[{i, j - 1}]) / grid.getDx().y;
            }
        }
    }
}
}  // namespace asa