#include <Eigen/Eigen>
#include <igl/grad.h>
#include <igl/local_basis.h>
#include <imgui.h>

#include "viewer_proxy.h"

/*** insert any necessary libigl headers here ***/
#include <igl/boundary_loop.h>
#include <igl/map_vertices_to_circle.h>
#include <igl/dijkstra.h>
#include <igl/adjacency_list.h>
#include <igl/adjacency_matrix.h>
#include <igl/cotmatrix.h>
#include <igl/sum.h>
#include <igl/diag.h>
#include <igl/cat.h>
#include <igl/doublearea.h>

using namespace std;
using namespace Eigen;
using Viewer = ViewerProxy;

#define CORE_3D viewer.core(1)
#define CORE_2D viewer.core(2)

// vertex array, #V x3
Eigen::MatrixXd V;

// face array, #F x3
Eigen::MatrixXi F;

// UV coordinates, #V x2
Eigen::MatrixXd UV;
const char *constraints[] = {"fixed boundary", "2 verts", "DOF"};
const char *distortion[] = {"Angle", "Length", "Area"};
enum {UNIT_DISK_BOUNDARY, TWO_VERTICES_POSITIONS, NECESSARY_DOF};
enum {ANGLE, LENGTH, AREA};
int selected_constraint = 0;
int selected_distortion = 0;
float TextureResolution = 10;
SparseMatrix<double> getAreaMatrix();
vector<int> pickTwoPoints();
void computeDistortion();
MatrixXd color;
bool calculateDistortion = false;

void Redraw(ViewerProxy& viewer) {
  // Update the mesh in the viewer.
  ViewerProxy::Data mesh_data = viewer.data(0);
  ViewerProxy::Data uv_mesh_data = viewer.data(1);
  mesh_data.clear();
  mesh_data.set_mesh(V, F);
  mesh_data.set_face_based(false);
  uv_mesh_data.clear();
  if (UV.size() != 0) {
    mesh_data.set_uv(TextureResolution * UV);
    mesh_data.show_texture = true;
    uv_mesh_data.set_mesh(UV, F);
    CORE_2D.align_camera_center(UV);
  }
  if (calculateDistortion) {
    mesh_data.set_colors(color);
    uv_mesh_data.set_colors(color);
    calculateDistortion = false;
  }
}

static void computeSurfaceGradientMatrix(SparseMatrix<double> &D1,
                                         SparseMatrix<double> &D2) {
  MatrixXd F1, F2, F3;
  SparseMatrix<double> DD, Dx, Dy, Dz;

  igl::local_basis(V, F, F1, F2, F3);
  igl::grad(V, F, DD);

  Dx = DD.topLeftCorner(F.rows(), V.rows());
  Dy = DD.block(F.rows(), 0, F.rows(), V.rows());
  Dz = DD.bottomRightCorner(F.rows(), V.rows());

  D1 = F1.col(0).asDiagonal() * Dx + F1.col(1).asDiagonal() * Dy +
       F1.col(2).asDiagonal() * Dz;
  D2 = F2.col(0).asDiagonal() * Dx + F2.col(1).asDiagonal() * Dy +
       F2.col(2).asDiagonal() * Dz;
}

static inline void SSVD2x2(const Eigen::Matrix2d &J, Eigen::Matrix2d &U,
                           Eigen::Matrix2d &S, Eigen::Matrix2d &V) {
  double e = (J(0) + J(3)) * 0.5;
  double f = (J(0) - J(3)) * 0.5;
  double g = (J(1) + J(2)) * 0.5;
  double h = (J(1) - J(2)) * 0.5;
  double q = sqrt((e * e) + (h * h));
  double r = sqrt((f * f) + (g * g));
  double a1 = atan2(g, f);
  double a2 = atan2(h, e);
  double rho = (a2 - a1) * 0.5;
  double phi = (a2 + a1) * 0.5;

  S(0) = q + r;
  S(1) = 0;
  S(2) = 0;
  S(3) = q - r;

  double c = cos(phi);
  double s = sin(phi);
  U(0) = c;
  U(1) = s;
  U(2) = -s;
  U(3) = c;

  c = cos(rho);
  s = sin(rho);
  V(0) = c;
  V(1) = -s;
  V(2) = s;
  V(3) = c;
}

void ConvertConstraintsToMatrixForm(const VectorXi &indices,
                                    const MatrixXd &positions,
                                    Eigen::SparseMatrix<double> &C,
                                    VectorXd &d) {
  // Convert the list of fixed indices and their fixed positions to a linear
  // system.
  // Hint: The matrix C should contain only one non-zero element per row
  // and d should contain the positions in the correct order.
    vector<Triplet<double>> triplets;
    if (selected_constraint != NECESSARY_DOF) {
        int num_constraints = positions.rows();
        d.resize(num_constraints * 2);
        C.resize(num_constraints * 2, V.rows() * 2);
        triplets.reserve(num_constraints * 2);
        for (int i = 0; i < num_constraints; ++i) {
            d[i] = positions(i, 0);
            d[i + num_constraints] = positions(i, 1);
            triplets.emplace_back(i, indices[i], 1);
            triplets.emplace_back(i + num_constraints, indices[i] + V.rows(), 1);
        }
    } else {
        //only using two D.O.F as constraints
        // u1 u2 of two vertexes
        d.resize(2);
        C.resize(2, V.rows() * 2);
        triplets.reserve(2);
        // U coordinate
        d[0] = positions(0, 0);
        d[1] = positions(1, 0);
        triplets.emplace_back(0, indices[0], 1);
        triplets.emplace_back(1, indices[1], 1);
    }
    C.setFromTriplets(triplets.begin(), triplets.end());
}

void computeParameterization(int type) {
  VectorXi fixed_UV_indices;
  MatrixXd fixed_UV_positions;

  SparseMatrix<double> A;
  VectorXd b;
  Eigen::SparseMatrix<double> C;
  VectorXd d;
  // Find the indices of the boundary vertices of the mesh and put them in
  // fixed_UV_indices
  switch (selected_constraint) {
      case UNIT_DISK_BOUNDARY: {
          // The boundary vertices should be fixed to positions on the unit disc. Find
          // these position and save them in the #V x 2 matrix fixed_UV_position.
          igl::boundary_loop(F, fixed_UV_indices);
          igl::map_vertices_to_circle(V, fixed_UV_indices, fixed_UV_positions);
          break;
      }

      case TWO_VERTICES_POSITIONS: {
          // Fix two UV vertices. This should be done in an intelligent way.
          // Hint: The two fixed vertices should be the two most distant one on the
          // mesh.
      }

      case NECESSARY_DOF: {
          // Add constraints for fixing only the necessary degrees of freedom for the
          // parameterization, avoiding an unnecessarily over-constrained system.x
          vector<int> indices = pickTwoPoints();
          fixed_UV_indices.resize(2);
          fixed_UV_indices << indices[0], indices[1];
          fixed_UV_positions.resize(2, 2);
          //set the UV coordinate randomly
          fixed_UV_positions << -1, 0,
                                 1, 0;
          break;
      }
  }

  ConvertConstraintsToMatrixForm(fixed_UV_indices, fixed_UV_positions, C, d);

  // Find the linear system for the parameterization (1- Tutte, 2- Harmonic, 3-
  // LSCM, 4- ARAP) and put it in the matrix A. The dimensions of A should be
  // 2#V x 2#V.
  int nv = V.rows();
  int N = C.rows() / 2;
  int nf = F.rows();

  if (type == '1') {
    // Add your code for computing uniform Laplacian for Tutte parameterization
    // Hint: use the adjacency matrix of the mesh
    Eigen::SparseMatrix<double> Adj, Lu, D;
    Eigen::SparseVector<double> Degree;
    igl::adjacency_matrix(F, Adj);
    // calculate the degree matrix
    igl::sum(Adj, 1, Degree);
    igl::diag(Degree, D);
    Lu = Adj - D;

    //build A.TA
    A.resize(2 * nv, 2 * nv);
    vector<Triplet<double>> triplets;
    for (int i = 0; i < Lu.outerSize(); ++i) {
        for (SparseMatrix<double>::InnerIterator it(Lu, i); it; ++it) {
            triplets.emplace_back(it.row(), it.col(), it.value());
            triplets.emplace_back(it.row() + nv, it.col() + nv, it.value());
        }
    }
    A.setFromTriplets(triplets.begin(), triplets.end());
    //A.Tb = 0
    b.setZero(2 * nv);
  }

  if (type == '2') {
    // Add your code for computing cotangent Laplacian for Harmonic
    // parameterization Use can use a function "cotmatrix" from libIGL, but
    // ~~~~***READ THE DOCUMENTATION***~~~~
    Eigen::SparseMatrix<double> Lc;
    igl::cotmatrix(V, F, Lc);

    //build A.TA
    A.resize(2 * nv, 2 * nv);
    vector<Triplet<double>> triplets;
    for (int i = 0; i < Lc.outerSize(); ++i) {
        for (SparseMatrix<double>::InnerIterator it(Lc, i); it; ++it) {
            triplets.emplace_back(it.row(), it.col(), it.value());
            triplets.emplace_back(it.row() + nv, it.col() + nv, it.value());
        }
    }
    A.setFromTriplets(triplets.begin(), triplets.end());
    //A.Tb = 0
    b.setZero(2 * nv);
  }

  if (type == '3') {
    // Add your code for computing the system for LSCM parameterization
    // Note that the libIGL implementation is different than what taught in the
    // tutorial! Do not rely on it!!
    //building triangle area matrix
    SparseMatrix<double> areaMatrix(nf, nf);
    areaMatrix = getAreaMatrix();
    SparseMatrix<double> Dx, Dy, DxT, DyT;
    //calculate gradient operator along local triangle coordinate axis
    computeSurfaceGradientMatrix(Dx, Dy);
    DxT = Dx.transpose();
    DyT = Dy.transpose();
    SparseMatrix<double> A11, A12, A21, A22, upA, downA;
    A11 = 2 * DxT * areaMatrix * Dx + 2 * DyT * areaMatrix * Dy;
    A12 = -2 * DxT * areaMatrix * Dy + 2 * DyT * areaMatrix * Dx;
    A21 = -2 * DyT * areaMatrix * Dx + 2 * DxT * areaMatrix * Dy;
    A22 = 2 * DyT * areaMatrix * Dy + 2 * DxT * areaMatrix * Dx;
    igl::cat(2, A11, A12, upA);
    igl::cat(2, A21, A22, downA);
    igl::cat(1, upA, downA, A);
    b.setZero(2 * nv);
  }

  if (type == '4') {
    // Add your code for computing ARAP system and right-hand side
    // Implement a function that computes the local step first
    // Then construct the matrix with the given rotation matrices
    //check UV and initiate
    if (UV.rows() == 0) {
        computeParameterization('3');
    }
    //building triangle area matrix
    SparseMatrix<double> areaMatrix(nf, nf);
    areaMatrix = getAreaMatrix();
    SparseMatrix<double> Dx, Dy, DxT, DyT;
    //calculate gradient operator along local triangle coordinate axis nf * nv
    computeSurfaceGradientMatrix(Dx, Dy);
    DxT = Dx.transpose();
    DyT = Dy.transpose();
    SparseMatrix<double> R(4 * nf, 1);
    vector<Triplet<double>> triplets;
    triplets.reserve(4 * nf);
    //calculate Jacobian for every face
    VectorXd U = UV.col(0);
    VectorXd V = UV.col(1);
    //nf * 1
    VectorXd J11 = Dx * U;
    VectorXd J12 = Dy * U;
    VectorXd J21 = Dx * V;
    VectorXd J22 = Dy * V;
    for (int i = 0; i < nf; ++i) {
        Matrix2d J, U, S, V, tmpS, VT, Ri;
        J.resize(2, 2);
        J << J11(i), J12(i), J21(i), J22(i);
        SSVD2x2(J, U, S, V);
        VT = V.transpose();
        double s = (U * VT).determinant();
        tmpS.resize(2, 2);
        tmpS << 1, 0, 0, s;
        Ri = U * tmpS * VT;
        triplets.emplace_back(i, 0, Ri(0, 0));
        triplets.emplace_back(i + nf, 0, Ri(0, 1));
        triplets.emplace_back(i + 2 * nf, 0, Ri(1, 0));
        triplets.emplace_back(i + 3 * nf, 0, Ri(1, 1));
    }
    R.setFromTriplets(triplets.begin(), triplets.end());
    SparseMatrix<double> A11, A22, upA, downA, zeroMatrix;
    zeroMatrix.resize(nv, nv);
    zeroMatrix.setZero();
    A11 = DxT * areaMatrix * Dx + DyT * areaMatrix * Dy;
    A22 = DxT * areaMatrix * Dx + DyT * areaMatrix * Dy;
    igl::cat(2, A11, zeroMatrix, upA);
    igl::cat(2, zeroMatrix, A22, downA);
    igl::cat(1, upA, downA, A);
    VectorXd upb, downb, R11, R12, R21, R22;
    R11 = R.block(0, 0, nf, 1);
    R12 = R.block(nf, 0, nf, 1);
    R21 = R.block(2 * nf, 0, nf, 1);
    R22 = R.block(3 * nf, 0, nf, 1);
    upb = DxT * areaMatrix * R11 + DyT * areaMatrix * R12;
    downb = DxT * areaMatrix * R21 + DyT * areaMatrix * R22;
    igl::cat(1, upb, downb, b);
  }

  // Solve the linear system.
  // Construct the system as discussed in class and the assignment sheet
  // Use igl::cat to concatenate matrices
  // Use Eigen::SparseLU to solve the system. Refer to tutorial 4 for more
  // details.

  SparseMatrix<double> upMatrix, downMatrix, zeroMatrix, lhs, CT;
  zeroMatrix.resize(2 * N, 2 * N);
  zeroMatrix.setZero();
  //A means A.TA
  CT = C.transpose();
  igl::cat(2, A, CT, upMatrix);
  igl::cat(2, C, zeroMatrix, downMatrix);
  igl::cat(1, upMatrix, downMatrix, lhs);
  VectorXd rhs;
  rhs.resize(2 * nv + 2 * N);
  //b means A.Tb
  rhs << b, d;

  //solver
  SparseLU<SparseMatrix<double>> solver;
  solver.analyzePattern(lhs);
  solver.factorize(lhs);
  if (solver.info() != Eigen::Success) {
      std::cerr << "Factorization failed: " << solver.lastErrorMessage() << std::endl;
      return;
  }

  VectorXd X = solver.solve(rhs);

  if (solver.info() != Eigen::Success) {
      // Handle the error, solving failed
      std::cerr << "Solving failed." << std::endl;
      return;
  }

  // Copy the solution to UV.
  UV.resize(nv, 2);
  //U
  UV.col(0) = X.block(0, 0, nv, 1);
  //V
  UV.col(1) = X.block(nv, 0, nv, 1);
}

bool callback_key_down(Viewer &viewer, unsigned char key, int modifiers) {
  switch (key) {
  case '1':
  case '2':
  case '3':
  case '4':
    computeParameterization(key);
    break;
  case '+':
    TextureResolution /= 2;
    break;
  case '-':
    TextureResolution *= 2;
    break;
  case 'D':
    computeDistortion();
    break;
  }
  Redraw(viewer);
  return true;
}

bool load_mesh(Viewer& viewer, string filename) {
  viewer.load_mesh(filename, V, F);
  viewer.core().align_camera_center(V);

  return true;
}

bool callback_init(Viewer &viewer) {
  viewer.append_core();
  CORE_2D.orthographic = true;
  CORE_2D.disable_rotation();

  // Main mesh.
  viewer.data().set_visible(false, CORE_2D.id);
  // UV mesh.
  viewer.append_mesh().set_visible(false, CORE_3D.id);

  Redraw(viewer);

  return false;
}

int main(int argc, char *argv[]) {
  ViewerProxy& viewer = ViewerProxy::get_instance();
  if (argc != 2) {
    cout << "Usage ex4_bin <mesh.off/obj>" << endl;
    load_mesh(viewer, "../data/cathead.obj");
  } else {
    // Read points and normals
    load_mesh(viewer, argv[1]);
  }

  ViewerProxy::Menu menu = viewer.menu();
  menu.callback_draw_viewer_menu = [&]() {
    // Draw parent menu content
    menu.draw_viewer_menu();

    // Add new group
    if (ImGui::CollapsingHeader("Parmaterization",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::Combo("Constraints", &selected_constraint, constraints,
                   IM_ARRAYSIZE(constraints));
      ImGui::Combo("Distortion", &selected_distortion, distortion,
                     IM_ARRAYSIZE(distortion));
      if (ImGui::SliderFloat("scale", &TextureResolution, 0, 40)) {
        Redraw(viewer);
      }

      // TODO: Add more parameters to tweak here...
    }
  };

  viewer.callback_key_down = callback_key_down;
  viewer.callback_init = callback_init;
  viewer.callback_post_resize = [&](Viewer &viewer, int w, int h) {
    CORE_2D.viewport = Eigen::Vector4f(0, 0, w / 2, h);
    CORE_3D.viewport = Eigen::Vector4f(w / 2, 0, w / 2, h);
    return false;
  };

  viewer.launch();
}

SparseMatrix<double> getAreaMatrix() {
    int nf = F.rows();
    VectorXd area;
    igl::doublearea(V, F, area);
    vector<Triplet<double>> triplets;
    //building triangle area matrix
    SparseMatrix<double> areaMatrix(nf, nf);
    for (int i = 0; i < area.size(); ++i) {
        double value = area(i);
        triplets.emplace_back(i, i, value);
    }
    areaMatrix.setFromTriplets(triplets.begin(), triplets.end());
    return areaMatrix;
}

void computeDistortion() {
    //set flag
    calculateDistortion = true;
//    VectorXd areas;
//    igl::doublearea(UV, F, areas);
//    UV /= sqrt(areas.sum() / 2);
    //calculate Jacobian for each face
    int nf = F.rows();
    color.resize(nf, 3);

    SparseMatrix<double> Dx, Dy;
    //calculate gradient operator along local triangle coordinate axis nf * nv
    computeSurfaceGradientMatrix(Dx, Dy);
    SparseMatrix<double> R(4 * nf, 1);
    vector<Triplet<double>> triplets;
    //calculate Jacobian for every face
    VectorXd U = UV.col(0);
    VectorXd V = UV.col(1);
    //nf * 1
    VectorXd J11 = Dx * U;
    VectorXd J12 = Dy * U;
    VectorXd J21 = Dx * V;
    VectorXd J22 = Dy * V;
    VectorXd distortionValues(nf);
    switch (selected_distortion) {
        case ANGLE:
            for (int i = 0; i < nf; ++i) {
                Matrix2d J, I;
                J.resize(2, 2);
                J << J11(i), J12(i), J21(i), J22(i);
                I.resize(2, 2);
                I.setIdentity();
                distortionValues(i) = (J + J.transpose() - J.trace() * I).squaredNorm();
            }
            break;
        case LENGTH:
            for (int i = 0; i < nf; ++i) {
                Matrix2d J, U, S, V, tmpS, VT, Ri;
                J.resize(2, 2);
                J << J11(i), J12(i), J21(i), J22(i);
                SSVD2x2(J, U, S, V);
                VT = V.transpose();
                double s = (U * VT).determinant();
                tmpS.resize(2, 2);
                tmpS << 1, 0, 0, s;
                Ri = U * tmpS * VT;
                distortionValues(i) = (J - Ri).squaredNorm();
            }
            break;
        case AREA:
            for (int i = 0; i < nf; ++i) {
                Matrix2d J;
                J.resize(2, 2);
                J << J11(i), J12(i), J21(i), J22(i);
                distortionValues(i) = pow(J.determinant() - 1, 2);
            }
            break;
    }
    //normalizing
    double min_distortion = distortionValues.minCoeff();
    double max_distortion = distortionValues.maxCoeff();
    distortionValues = (distortionValues.array() - min_distortion) / (max_distortion - min_distortion);
    //from white (1, 1, 1) to red(1, 0, 0)
    VectorXd ones(nf);
    ones.setOnes();
    color.col(0) = ones;
    color.col(1) = ones - distortionValues;
    color.col(2) = ones - distortionValues;
}

vector<int> pickTwoPoints() {
    double max_dist = 0;
    //source and target index
    int v1 = 0, v2 = 0;
    //to store the min_dist from the source to target vertex
    VectorXd min_dist;
    //store the previous vertex indices
    VectorXi prev;
    //store the target vertex indeces
    set<int> tar;
    //adjacent vertex
    vector<vector<int>> VV;
    igl::adjacency_list(F, VV);

    for (int i = 0; i < V.rows(); ++i) {
        igl::dijkstra(i, tar, VV, min_dist, prev);
        //find the maximum in the min_dist
        for (int j = 0; j < min_dist.size(); ++j) {
            if (min_dist[j] > max_dist) {
                max_dist = min_dist[j];
                v1 = i;
                v2 = j;
            }
        }
    }
    vector<int> res;
    res.push_back(v1);
    res.push_back(v2);
    return  res;
}