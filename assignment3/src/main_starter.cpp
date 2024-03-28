#include <iostream>
#include <igl/read_triangle_mesh.h>
#include <imgui.h>
#include "viewer_proxy.h"

#include <igl/jet.h>
#include <igl/gaussian_curvature.h>
#include <igl/invert_diag.h>
#include <igl/sum.h>
#include <igl/speye.h>
#include <igl/bfs.h>
#include <igl/cotmatrix.h>
#include <igl/principal_curvature.h>
#include <igl/barycenter.h>
#include <igl/knn.h>
#include <igl/octree.h>
/*** insert any libigl headers here ***/
#include<igl/per_vertex_normals.h>
#include<igl/fit_plane.h>


using namespace std;
using namespace Eigen;
using namespace igl;
using Viewer = ViewerProxy;


// Vertex array, #Vx3
Eigen::MatrixXd V;
// Face array, #Fx3
Eigen::MatrixXi F;
//Face normals #Fx3
Eigen::MatrixXd FN;
//Vertex normals #Vx3
Eigen::MatrixXd VN;

// Per-vertex uniform normal array, #Vx3
Eigen::MatrixXd N_uniform;
// Per-vertex area-weighted normal array, #Vx3
Eigen::MatrixXd N_area;
// Per-vertex mean-curvature normal array, #Vx3
Eigen::MatrixXd N_meanCurvature;
// Per-vertex PCA normal array, #Vx3
Eigen::MatrixXd N_PCA;
// Per-vertex quadratic fitted normal array, #Vx3
Eigen::MatrixXd N_quadraticFit;

// Per-vertex mean curvature, #Vx3
Eigen::VectorXd K_mean;
// Per-vertex Gaussian curvature, #Vx3
Eigen::VectorXd K_Gaussian;
// Per-vertex minimal principal curvature, #Vx3
Eigen::VectorXd K_min_principal;
// Per-vertex maximal principal curvature, #Vx3
Eigen::VectorXd K_max_principal;
// Per-vertex color array, #Vx3
Eigen::MatrixXd colors_per_vertex;

// Explicitely smoothed vertex array, #Vx3
Eigen::MatrixXd V_expLap;
// Implicitely smoothed vertex array, #Vx3
Eigen::MatrixXd V_impLap;
// Bilateral smoothed vertex array, #Vx3
Eigen::MatrixXd V_bilateral;

RowVector3d fit_and_get_normal(MatrixXd& neighbors);

bool isDiagonal(const SparseMatrix<double> &M);

void normalize(MatrixXd& V);

RowVector3d fit_and_get_normal(MatrixXd& neighbors) {
    RowVector3d mean = neighbors.colwise().mean();
    neighbors = neighbors.rowwise() - mean;
    int k = neighbors.rows();
    MatrixXd covariance = 1 / (k - 1.0) * neighbors.transpose() * neighbors;
    SelfAdjointEigenSolver<MatrixXd> eigen(covariance);
    MatrixXd eigenvectors = eigen.eigenvectors().reverse();
    //translate into local frame : (x, y, z)
    MatrixXd local_neighbors = neighbors * eigenvectors;
    //fit
    MatrixXd A(k,5);
    MatrixXd b(k,1);
    MatrixXd sol(5,1);

    for(int c = 0; c < k; ++c)
    {
        double u = local_neighbors(c, 0);
        double v = local_neighbors(c, 1);
        double n = local_neighbors(c, 2);

        A(c,0) = u*u;
        A(c,1) = v*v;
        A(c,2) = u*v;
        A(c,3) = u;
        A(c,4) = v;

        b(c) = n;
    }

    sol = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    RowVector3d normal;
    //(d, e, -1)
    normal << sol(3), sol(4), -1;
    //rotate back to original frame
    normal *= eigenvectors.transpose();
    return normal;
}

bool isDiagonal(const SparseMatrix<double> &M) {
    for (int k = 0; k < M.outerSize(); ++k) {
        for (SparseMatrix<double>::InnerIterator it(M, k); it; ++it) {
            if (it.row() != it.col() && it.value() != 0) {
                return false;
            }
        }
    }
    return true;
}

void normalize(MatrixXd& V) {
    // normalizing
    // Compute centroid and subtract (also important for numerics)
    VectorXd dblA;
    doublearea(V, F, dblA);
    double area = 0.5 * dblA.sum();
    MatrixXd BC;
    barycenter(V, F, BC);
    RowVector3d centroid(0,0,0);
    for(int i = 0;i < BC.rows();i++)
    {
        centroid += 0.5 * dblA(i) / area * BC.row(i);
    }
    V.rowwise() -= centroid;
    // Normalize to unit surface area (important for numerics)
    V.array() /= sqrt(area);
}


bool callback_key_down(Viewer& viewer, unsigned char key, int modifiers) {
    if (key == '1') {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        // Add your code for computing uniform vertex normals here:
        // store in N_uniform
        N_uniform.resize(V.rows(), 3);
        for (int i = 0; i < F.rows(); ++i) {
            int v0_index = F(i, 0);
            int v1_index = F(i, 1);
            int v2_index = F(i, 2);
            Vector3d v0 = V.row(v0_index);
            Vector3d v1 = V.row(v1_index);
            Vector3d v2 = V.row(v2_index);

            Vector3d normal = (v1 - v0).cross(v2 - v0).normalized();
            N_uniform.row(v0_index) += normal;
            N_uniform.row(v1_index) += normal;
            N_uniform.row(v2_index) += normal;
        }

        // Use igl::per_vertex_normals to orient your normals consistently (i.e. to choose consistent signs for the normals).
        per_vertex_normals(V, F, PER_VERTEX_NORMALS_WEIGHTING_TYPE_UNIFORM, VN);
        //orient the normals consistently
        // if using the standard counter clock order, there's no need to orient
        for (int i = 0; i < N_uniform.rows(); ++i) {
            if (N_uniform.row(i).dot(VN.row(i)) < 0) {
                N_uniform.row(i) *= -1;
            }
        }
        // Set the viewer normals.
        N_uniform.rowwise().normalize();
        viewer.data().set_normals(N_uniform);
    }

    if (key == '2') {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        // Add your code for computing area-weighted vertex normals here:
        // store in N_area
        N_area.resize(V.rows(), 3);
        for (int i = 0; i < F.rows(); ++i) {
            int v0_index = F(i, 0);
            int v1_index = F(i, 1);
            int v2_index = F(i, 2);
            Vector3d v0 = V.row(v0_index);
            Vector3d v1 = V.row(v1_index);
            Vector3d v2 = V.row(v2_index);

            Vector3d normal = (v1 - v0).cross(v2 - v0);
            double area = 0.5 * normal.norm();
            normal = normal.normalized();
            N_area.row(v0_index) += area * normal;
            N_area.row(v1_index) += area * normal;
            N_area.row(v2_index) += area * normal;
        }
        // Use igl::per_vertex_normals to orient your normals consistently (i.e. to choose consistent signs for the normals).
        per_vertex_normals(V, F, PER_VERTEX_NORMALS_WEIGHTING_TYPE_AREA, VN);
        for (int i = 0; i < N_area.rows(); ++i) {
            if (N_area.row(i).dot(VN.row(i)) < 0) {
                N_area.row(i) *= -1;
            }
        }
        // Set the viewer normals.
        N_area.rowwise().normalize();
        viewer.data().set_normals(N_area);
    }

    if (key == '3') {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        // Add your code for computing mean-curvature vertex normals here:
        // store in N_meanCurvature
        N_meanCurvature.resize(V.rows(), 3);
        //calculate Laplacian matrix
        //L = M-1 * Lc
        SparseMatrix<double> M_inv;
        SparseMatrix<double> Lc, L;
        SparseMatrix<double> M;
        cotmatrix(V, F, Lc);
        massmatrix(V, F, MASSMATRIX_TYPE_BARYCENTRIC, M);
        invert_diag(M,M_inv);
        assert(isDiagonal(M));
        assert(isDiagonal(M_inv));
        L = M_inv * Lc;
        N_meanCurvature = -L * V;
        // Use igl::per_vertex_normals to orient your normals consistently (i.e. to choose consistent signs for the normals).
        per_vertex_normals(V, F, PER_VERTEX_NORMALS_WEIGHTING_TYPE_ANGLE, VN);
        for (int i = 0; i < N_meanCurvature.rows(); ++i) {
            if (N_meanCurvature.row(i).dot(VN.row(i)) < 0) {
                N_meanCurvature.row(i) *= -1;
            }
        }
        // Set the viewer normals.
        N_meanCurvature.rowwise().normalize();
        viewer.data().set_normals(N_meanCurvature);
    }

    if (key == '4') {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        // Add your code for computing PCA vertex normals here:
        // store in N_PCA
        N_PCA.resize(V.rows(), 3);

        //buidling octree
        vector<vector<int>> O_PI;
        MatrixXi O_CH;
        MatrixXd O_CN;
        VectorXd O_W;
        octree(V, O_PI, O_CH, O_CN, O_W);

        int k = 50;
        MatrixXi I;
        knn(V, k, O_PI, O_CH, O_CN, O_W, I);
        for (int i = 0; i < I.rows(); ++i) {
            RowVectorXi neighbors_index = I.row(i);
            MatrixXd neighbors(neighbors_index.size(), 3);
            for (int j = 0; j < neighbors_index.size(); ++j) {
                neighbors.row(j) = V.row(neighbors_index[j]);
            }

            RowVector3d normal;
            RowVector3d origin;
            fit_plane(neighbors, normal, origin);
            N_PCA.row(i) = normal;
        }
        // Use igl::per_vertex_normals to orient your normals consistently (i.e. to choose consistent signs for the normals).
        per_vertex_normals(V, F, VN);
        for (int i = 0; i < N_PCA.rows(); ++i) {
            if (N_PCA.row(i).dot(VN.row(i)) < 0) {
                N_PCA.row(i) *= -1;
            }
        }
        // Set the viewer normals.
        N_PCA.rowwise().normalize();
        viewer.data().set_normals(N_PCA);
    }

    if (key == '5') {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        // Add your code for computing quadratic fitted vertex normals here:
        // store in N_quadraticFit
        N_quadraticFit.resize(V.rows(), 3);
        //buidling octree
        vector<vector<int>> O_PI;
        MatrixXi O_CH;
        MatrixXd O_CN;
        VectorXd O_W;
        octree(V, O_PI, O_CH, O_CN, O_W);

        int k = 50;
        MatrixXi I;
        knn(V, k, O_PI, O_CH, O_CN, O_W, I);
        for (int i = 0; i < I.rows(); ++i) {
            RowVectorXi neighbors_index = I.row(i);
            MatrixXd neighbors(neighbors_index.size(), 3);
            for (int j = 0; j < neighbors_index.size(); ++j) {
                neighbors.row(j) = V.row(neighbors_index[j]);
            }
            N_quadraticFit.row(i) = fit_and_get_normal(neighbors);
        }
        // Use igl::per_vertex_normals to orient your normals consistently (i.e. to choose consistent signs for the normals).
        per_vertex_normals(V, F, VN);
        for (int i = 0; i < N_quadraticFit.rows(); ++i) {
            if (N_quadraticFit.row(i).dot(VN.row(i)) < 0) {
                N_quadraticFit.row(i) *= -1;
            }
        }
        // Set the viewer normals.
        N_quadraticFit.rowwise().normalize();
        viewer.data().set_normals(N_quadraticFit);
    }

    if (key == '6') {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        colors_per_vertex.setZero(V.rows(),3);
        // Add your code for computing the discrete mean curvature:
        // store in K_mean
        MatrixXd PD1, PD2;
        VectorXd PV1, PV2;
        principal_curvature(V, F, PD1, PD2, PV1, PV2);
        K_mean = (PV1 + PV2) / 2.0;
        // For visualization, better to normalize the range of K_mean with the maximal and minimal curvatures.
        // store colors in colors_per_vertex
        jet(K_mean, true, colors_per_vertex);
        // Set the viewer colors
        viewer.data().set_colors(colors_per_vertex);
    }

    if (key == '7') {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        colors_per_vertex.setZero(V.rows(),3);
        // Add your code for computing the discrete Gaussian curvature:
        // store in K_Gaussian
        K_Gaussian.resize(V.rows());
        MatrixXd PD1, PD2;
        VectorXd PV1, PV2;
        principal_curvature(V, F, PD1, PD2, PV1, PV2);
        for (int i = 0; i < V.rows(); ++i) {
            K_Gaussian[i] = PV1[i] * PV2[i];
        }
        // For visualization, better to normalize the range of K_Gaussian with the maximal and minimal curvatures.
        // store colors in colors_per_vertex
        jet(K_Gaussian, true, colors_per_vertex);
        // Set the viewer colors
        viewer.data().set_colors(colors_per_vertex);
    }

    if (key == '8') {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        colors_per_vertex.setZero(V.rows(),3);
        // Add your code for computing the discrete minimal principal curvature:
        // store in K_min_principal
        K_max_principal, K_min_principal;
        VectorXd PV_max, PV_min;
        principal_curvature(V, F, K_max_principal, K_min_principal, PV_max, PV_min);
        // For visualization, better to normalize the range of K_min_principal with the maximal and minimal curvatures.
        // store colors in colors_per_vertex
        jet(PV_min, true, colors_per_vertex);
        // Uncomment the code below to draw a blue segment parallel to the minimal curvature direction,
        
         const double avg = igl::avg_edge_length(V,F);
         Eigen::Vector3d blue(0.2,0.2,0.8);
         viewer.data().add_edges(V + K_min_principal*avg, V - K_min_principal*avg, blue);
        
        // Set the viewer colors
        viewer.data().set_colors(colors_per_vertex);
    }

    if (key == '9') {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        colors_per_vertex.setZero(V.rows(),3);
        // Add your code for computing the discrete maximal principal curvature:
        // store in K_max_principal
        K_max_principal, K_min_principal;
        VectorXd PV_max, PV_min;
        principal_curvature(V, F, K_max_principal, K_min_principal, PV_max, PV_min);
        // For visualization, better to normalize the range of K_max_principal with the maximal and minimal curvatures
        // store colors in colors_per_vertex
        jet(PV_max, true, colors_per_vertex);
        // Uncomment the code below to draw a red segment parallel to the maximal curvature direction
        
         const double avg = igl::avg_edge_length(V,F);
         Eigen::Vector3d red(0.8,0.2,0.2);
         viewer.data().add_edges(V + K_max_principal*avg, V - K_max_principal*avg, red);
        
        // Set the viewer colors
        viewer.data().set_colors(colors_per_vertex);
    }

    if (key == 'E') {
        // Add your code for computing explicit Laplacian smoothing here:
        // store the smoothed vertices in V_expLap
        if (V_expLap.rows() == 0) {
            V_expLap = V;
        }

        SparseMatrix<double> Lc;
        SparseMatrix<double> M, M_inv, L;
        cotmatrix(V_expLap, F, Lc);
        massmatrix(V_expLap, F, MASSMATRIX_TYPE_BARYCENTRIC, M);
        invert_diag(M, M_inv);
        assert(isDiagonal(M));
        assert(isDiagonal(M_inv));
        L = M_inv * Lc;
        double dt = 0.0001;
        int n = V.rows();
        V_expLap = (MatrixXd::Identity(n, n) + dt * L) * V_expLap;

        //normalize
        normalize(V_expLap);

        // Set the smoothed mesh
        viewer.data().clear();
        viewer.data().set_mesh(V_expLap, F);
    }

    if (key == 'D'){
        // Implicit smoothing for comparison
        // store the smoothed vertices in V_impLap
        if (V_impLap.rows() == 0) {
            V_impLap = V;
        }

        SparseMatrix<double> Lc;
        SparseMatrix<double> M;
        cotmatrix(V_impLap, F, Lc);
        massmatrix(V_impLap, F, MASSMATRIX_TYPE_BARYCENTRIC, M);
        // Solve (M-delta*Lc) V = M*V
        double dt = 0.001;
        const auto & S = (M - dt * Lc);
        SimplicialLLT<Eigen::SparseMatrix<double>> solver(S);
        assert(solver.info() == Eigen::Success);
        V_impLap = solver.solve(M * V_impLap).eval();

        //normalize
        normalize(V_impLap);

        // Set the smoothed mesh
        viewer.data().clear();
        viewer.data().set_mesh(V_impLap, F);
    }

    if (key == 'B') {
        // Add your code for computing bilateral smoothing here:
        // store the smoothed vertices in V_bilateral
        // be careful of the sign mistake in the paper
        // use v' = v - n * (sum / normalizer) to update
        if (V_bilateral.rows() == 0) {
            V_bilateral = V;
        }
        //find neighbors
        vector<vector<int>> VV;
        adjacency_list(F, VV);
        //TODO?
        double sigma_c = 0.5;
        double sigma_s = 0.5;
        MatrixXd new_V_bilateral(V_bilateral.rows(), 3);
        for (int i = 0; i < V_bilateral.rows(); ++i) {
            double sum = 0;
            double normalizer = 0;
            vector<int> neighbors = VV[i];
            RowVector3d Vi = V_bilateral.row(i);
            RowVector3d normal = VN.row(i);
            double t, h, Wc, Ws;
            for (int j : neighbors) {
                RowVector3d Vj = V_bilateral.row(j);
                t = (Vi - Vj).norm();
                h = normal.dot(Vi - Vj);
                Wc = exp(-pow(t, 2) / (2 * pow(sigma_c, 2)));
                Ws = exp(-pow(h, 2) / (2 * pow(sigma_s, 2)));
                sum += (Wc * Ws) * h;
                normalizer += Wc * Ws;
            }
            new_V_bilateral.row(i) = Vi - (sum / normalizer) * normal;
        }

        V_bilateral = new_V_bilateral;

        //normalizing
        normalize(V_bilateral);

        // Set the smoothed mesh
        viewer.data().clear();
        viewer.data().set_mesh(V_bilateral, F);
    }


    return true;
}

bool load_mesh(Viewer& viewer,string filename, Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    igl::read_triangle_mesh(filename, V, F);
    viewer.data().clear();
    viewer.data().set_mesh(V,F);
    viewer.data().compute_normals();
    viewer.core().align_camera_center(V, F);
    return true;
}

int main(int argc, char *argv[]) {
    // Show the mesh
    Viewer& viewer = Viewer::get_instance();
    Viewer::Menu& menu = viewer.menu();
    viewer.callback_key_down = callback_key_down;

    std::string filename;
    if (argc == 2) {
        filename = std::string(argv[1]);
    }
    else {
        filename = std::string("../data/bumpy-cube.obj");
    }
    load_mesh(viewer,filename,V,F);

    callback_key_down(viewer, '1', 0);
    
    viewer.launch();
}
