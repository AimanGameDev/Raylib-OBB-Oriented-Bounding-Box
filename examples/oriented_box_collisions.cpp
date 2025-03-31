#include "raylib.h"
#include "raymath.h"
#include "obb.h"

int main(void)
{
    const int screenWidth = 800;
    const int screenHeight = 450;

    InitWindow(screenWidth, screenHeight, "raylib [models] example - oriented box collisions");

    Camera camera = {{0.0f, 10.0f, 10.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, 45.0f, 0};

    Vector3 playerPosition = {0.0f, 1.0f, 2.0f};
    Quaternion playerRotation = QuaternionIdentity();
    Vector3 playerSize = {1.0f, 2.0f, 1.0f};
    Color playerColor = GREEN;
    Mesh playerMesh = GenMeshCube(1.0, 1.0, 1.0);
    Model playerModel = LoadModelFromMesh(playerMesh);

    Vector3 enemyBoxPos = {-4.0f, 1.0f, 0.0f};
    Quaternion enemyBoxRotation = QuaternionIdentity();
    Vector3 enemyBoxSize = {2.0f, 2.0f, 2.0f};
    Mesh enemyBoxMesh = GenMeshCube(1.0, 1.0, 1.0);
    Model enemyBoxModel = LoadModelFromMesh(enemyBoxMesh);

    Vector3 enemySpherePos = {4.0f, 0.0f, 0.0f};
    float enemySphereSize = 1.5f;

    bool collision = false;

    SetTargetFPS(60);
    //--------------------------------------------------------------------------------------

    // Main game loop
    while (!WindowShouldClose())
    {
        // Update
        //----------------------------------------------------------------------------------

        // Move player
        if (IsKeyDown(KEY_RIGHT))
            playerPosition.x += 0.2f;
        else if (IsKeyDown(KEY_LEFT))
            playerPosition.x -= 0.2f;
        else if (IsKeyDown(KEY_DOWN))
            playerPosition.z += 0.2f;
        else if (IsKeyDown(KEY_UP))
            playerPosition.z -= 0.2f;

        collision = false;

        Quaternion rotation = QuaternionFromEuler(0.0, GetTime(), 0.0);
        playerRotation = rotation;
        enemyBoxRotation = rotation;

        Matrix playerModelMatrix = MatrixIdentity();
        playerModelMatrix = MatrixMultiply(playerModelMatrix, MatrixScale(playerSize.x, playerSize.y, playerSize.z));
        playerModelMatrix = MatrixMultiply(playerModelMatrix, QuaternionToMatrix(playerRotation));
        playerModelMatrix = MatrixMultiply(playerModelMatrix, MatrixTranslate(playerPosition.x, playerPosition.y, playerPosition.z));
        playerModel.transform = playerModelMatrix;

        Matrix enemyBoxModelMatrix = MatrixIdentity();
        enemyBoxModelMatrix = MatrixMultiply(enemyBoxModelMatrix, MatrixScale(enemyBoxSize.x, enemyBoxSize.y, enemyBoxSize.z));
        enemyBoxModelMatrix = MatrixMultiply(enemyBoxModelMatrix, QuaternionToMatrix(enemyBoxRotation));
        enemyBoxModelMatrix = MatrixMultiply(enemyBoxModelMatrix, MatrixTranslate(enemyBoxPos.x, enemyBoxPos.y, enemyBoxPos.z));
        enemyBoxModel.transform = enemyBoxModelMatrix;

        OBB playerObb = OBB();
        playerObb.center = playerPosition;
        playerObb.rotation = playerRotation;
        playerObb.halfExtents = Vector3Scale(playerSize, 0.5f);

        OBB enemyBoxObb = OBB();
        enemyBoxObb.center = enemyBoxPos;
        enemyBoxObb.rotation = enemyBoxRotation;
        enemyBoxObb.halfExtents = Vector3Scale(enemyBoxSize, 0.5f);

        if (CheckCollisionOBBvsOBB(&playerObb, &enemyBoxObb))
        {
            collision = true;
        }

        if (CheckCollisionSphereVsOBB(enemySpherePos, enemySphereSize, &playerObb))
        {
            collision = true;
        }

        Ray mouseRay = GetScreenToWorldRay(GetMousePosition(), camera);
        RayCollision rayCollision = GetRayCollisionOBB(mouseRay, &enemyBoxObb);

        if (collision)
            playerColor = RED;
        else
            playerColor = GREEN;
        //----------------------------------------------------------------------------------

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

        ClearBackground(RAYWHITE);

        BeginMode3D(camera);

        // Draw enemy-box
        DrawModel(enemyBoxModel, Vector3Zero(), 1.0, GRAY);
        OBB_DrawWireframe(&enemyBoxObb, DARKGRAY);

        // Draw enemy-sphere
        DrawSphere(enemySpherePos, enemySphereSize, GRAY);
        DrawSphereWires(enemySpherePos, enemySphereSize, 16, 16, DARKGRAY);

        // Draw player
        DrawModel(playerModel, Vector3Zero(), 1.0, playerColor);

        if (rayCollision.hit)
        {
            DrawPoint3D(rayCollision.point, RED);
            DrawLine3D(rayCollision.point, rayCollision.point + Vector3Scale(rayCollision.normal, rayCollision.distance), ORANGE);
        }

        DrawGrid(10, 1.0f); // Draw a grid

        EndMode3D();

        DrawText("Move player with arrow keys to collide", 220, 40, 20, GRAY);
        DrawText("Place mouse on the grey box to test raycast", 180, 60, 20, GRAY);

        DrawFPS(10, 10);

        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    // De-Initialization
    UnloadModel(playerModel);
    UnloadModel(enemyBoxModel);
    //--------------------------------------------------------------------------------------
    CloseWindow(); // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}
