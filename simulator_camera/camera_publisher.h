#pragma once
#include <mujoco/mujoco.h>

// update snapshots the scene under the simulator mutex. Rendering and publishing
// run afterwards on the UI thread with its current OpenGL context.
void g1_camera_reset();
void g1_camera_update(const mjModel* model, mjData* data);
void g1_camera_render(mjrContext* context);
