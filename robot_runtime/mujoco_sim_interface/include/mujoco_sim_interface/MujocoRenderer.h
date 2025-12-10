/* Computational Legged Robots: Planning and Control  */

#pragma once

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include "mujoco_sim_interface/MujocoUtils.h"
#include "robot_core/FPSTracker.h"

namespace robot::mujoco_sim_interface {

class MujocoSimInterface;

class MjState;

class MujocoRenderer {
 public:
  MujocoRenderer(const MujocoSimInterface* simInterface);

  ~MujocoRenderer();

  bool ok() const;

  void launchRenderThread();

  void waitForInit() const;

 private:
  /// These callbacks are required to be static by glfw3 and are hence not part of the visualizer class. They have access to the visualizer
  /// through the window user pointer.

  // keyboard callback
  static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);

  // mouse button callback
  static void mouse_button(GLFWwindow* window, int button, int act, int mods);

  // mouse move callback
  static void mouse_move(GLFWwindow* window, double xpos, double ypos);

  // scroll callback
  static void scroll(GLFWwindow* window, double xoffset, double yoffset);

  ///

  void setTransparency(float transparency) const;

  void renderLoop();

  void renderExternalForces();

  // Init must occur in the same thread that uses the opengl context.
  void initialize();

  // Cleanup must occur in same thread that owns the opengl context.
  void cleanup();

  const MujocoSimInterface* simInterface_;
  MjState simState_;

  std::thread render_thread_;

  GLFWwindow* window_;
  mjrRect viewport_ = {0, 0, 0, 0};

  int viewportWidth{1920};
  int viewportHeight{1024};

  // mouse interaction
  bool button_left = false;
  bool button_middle = false;
  bool button_right = false;

  double lastx = 0;
  double lasty = 0;

  double lastclicktm = 0;
  bool model_transparent = false;

  // Mujoco visualization structures
  mjvCamera mujocoCam_;       // abstract camera
  mjvOption mujocoOptions_;   // visualization options
  mjvScene mujocoScene_;      // abstract scene
  mjrContext mujocoContext_;  // custom GPU context

  size_t timeStepMicro_;

  std::atomic<bool> window_closed_{false};
  std::atomic<bool> init_complete_{false};

  FPSTracker rendererFps_{"renderer"};
};

}  // namespace robot::mujoco_sim_interface
