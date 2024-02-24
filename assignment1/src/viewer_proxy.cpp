#include "viewer_proxy.h"

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>

igl::opengl::glfw::Viewer viewer;
igl::opengl::glfw::imgui::ImGuiPlugin plugin;
igl::opengl::glfw::imgui::ImGuiMenu menu;
ViewerProxy* ViewerProxy::_instance = nullptr;

static void init_igl_viewer() {
  viewer.plugins.push_back(&plugin);
  plugin.widgets.push_back(&menu);
}

ViewerProxy& ViewerProxy::get_instance() {
  if (_instance == nullptr) {
    _instance = new ViewerProxy();
    init_igl_viewer();

    viewer.callback_key_down = [&](igl::opengl::glfw::Viewer& viewer,
                                   unsigned char key, int modifiers) {
      return _instance->callback_key_down(*_instance, key, modifiers);
    };
  }
  return *_instance;
}

void ViewerProxy::launch() { viewer.launch(); }

void ViewerProxy::Data::clear() { viewer.data().clear(); }
void ViewerProxy::Data::compute_normals() { viewer.data().compute_normals(); }
void ViewerProxy::Data::set_mesh(const Eigen::MatrixXd& V,
                                   const Eigen::MatrixXi& F) {
  viewer.data().set_mesh(V, F);
}
void ViewerProxy::Data::set_normals(const Eigen::MatrixXd& N) {
  viewer.data().set_normals(N);
}
void ViewerProxy::Data::set_colors(const Eigen::MatrixXd& colors) {
  viewer.data().set_colors(colors);
}

void ViewerProxy::Core::align_camera_center(const Eigen::MatrixXd& V,
                                              const Eigen::MatrixXi& F) {
  viewer.core().align_camera_center(V, F);
}
