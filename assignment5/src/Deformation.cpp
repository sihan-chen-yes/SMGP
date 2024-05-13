#include "Deformation.h"
#include <igl/cotmatrix.h>
#include <igl/massmatrix.h>
#include <igl/invert_diag.h>
#include <igl/slice.h>
#include <igl/slice_into.h>
#include <igl/per_vertex_normals.h>
#include <igl/adjacency_list.h>

using namespace Eigen;
using namespace igl;
using namespace std;

void Deformation::update_handle_vertex_selection(const Eigen::VectorXi &new_handle_id,
                                                 const Eigen::VectorXi &new_handle_vertices)
{
    // handle_id: the vertex-to-handle index, #V x1 (-1 if vertex is free)
    // handle_vertices: list of all vertices belonging to handles, #HV x1
    // Add your code for updating the handle vertex selection here ...

    //cholesky decomposition to compute Aff Afc
    SparseMatrix<double> Lw, M, M_inv, Aff, A;
    cotmatrix(V_original, F, Lw);
    massmatrix(V_original, F, MASSMATRIX_TYPE_DEFAULT, M);
    invert_diag(M, M_inv);

    A = Lw * M_inv * Lw;

    new_free_vertices.resize(V_original.rows() - new_handle_vertices.rows());
    int cnt = 0;
    for (int i = 0; i < V_original.rows(); ++i) {
        if (new_handle_id[i] == -1) {
            //free vertices
            new_free_vertices[cnt++] = i;
        }
    }
    slice(A, new_free_vertices, new_free_vertices, Aff);
    slice(A, new_free_vertices, new_handle_vertices, Afc);

    solver.compute(Aff);
    this->new_handle_vertices = new_handle_vertices;
}

void Deformation::get_smooth_mesh(Eigen::MatrixXd &V_res) {
    // Get the smooth mesh B
    // Store the result to V_res
    Eigen::MatrixXd rhs, V_f, V_c;
    slice(V_original, new_handle_vertices, 1, V_c);
    rhs = -Afc * V_c;
    V_f = solver.solve(rhs);
    slice_into(V_f, new_free_vertices, 1, V_res);

    //displacement between S and B
    MatrixXd displacement = V_original - V_res;
    //displacements decomposition to get component
    component.resize(V_res.rows(), 3);
    MatrixXd N;
    Vector3d Ni, vi, vj, x_axis, y_axis;
    per_vertex_normals(V_res, F, N);
    vector<vector<int>> neighbors;
    adjacency_list(F, neighbors);

    //correspondent longest edge
    v_index.resize(V_res.rows());
    //building local frame
    for (int i = 0; i < V_res.rows(); ++i) {
        Ni = N.row(i);
        vi = V_res.row(i);
        //search max projection length
        double p_max = 0;
        Vector3d current_projection;
        //search neighbors
        for (int j : neighbors[i]) {
            vj = V_res.row(j);
            current_projection = (vj - vi) - Ni.dot(vj - vi) * Ni;
            double p = current_projection.squaredNorm();
            if (p > p_max) {
                v_index[i] = j;
                p_max = p;
                x_axis = current_projection.normalized();
            }
        }
        y_axis = Ni.cross(x_axis);
        //projection to get xyz components
        component(i, 0) = displacement.row(i).dot(x_axis);
        component(i, 1) = displacement.row(i).dot(y_axis);
        component(i, 2) = displacement.row(i).dot(Ni);
    }
}

void Deformation::get_deformed_smooth_mesh(const Eigen::MatrixXd &handle_vertex_positions, Eigen::MatrixXd &V_res) {
    // Given the handle vertex positions, get the deformed smooth mesh B'
    // Store the result to V_res
    // B'
    Eigen::MatrixXd rhs, V_f;
    rhs = -Afc * handle_vertex_positions;
    V_f = solver.solve(rhs);
    slice_into(V_f, new_free_vertices, 1, V_res);
    slice_into(handle_vertex_positions, new_handle_vertices, 1, V_res);
}

void Deformation::get_deformed_mesh(const Eigen::MatrixXd &handle_vertex_positions, Eigen::MatrixXd &V_res) {
    // Given the handle vertex positions, get the deformed mesh with details S'
    // Store the result to V_res
    // B'
    get_deformed_smooth_mesh(handle_vertex_positions, V_res);

    //add details back
    MatrixXd N_prime, displacement_prime(V_res.rows(), 3);
    Vector3d N_i_prime, vi_prime, vj_prime, x_axis_prime, y_axis_prime;
    per_vertex_normals(V_res, F, N_prime);
    for (int i = 0; i < V_res.rows(); ++i) {
        N_i_prime = N_prime.row(i);
        vi_prime = V_res.row(i);
        vj_prime = V_res.row(v_index[i]);
        x_axis_prime = ((vj_prime - vi_prime) - N_i_prime.dot(vj_prime - vi_prime) * N_i_prime).normalized();
        y_axis_prime = N_i_prime.cross(x_axis_prime);
        displacement_prime.row(i) = component(i, 0) * x_axis_prime + component(i, 1) * y_axis_prime + component(i, 2) * N_i_prime;
    }
    //S' = B' + D
    V_res = V_res + displacement_prime;
}

void Deformation::get_deformed_mesh_deformation_transfer(const Eigen::MatrixXd &handle_vertex_positions, Eigen::MatrixXd &V_res)
{
    // Implement deformation transfer here.
    // Store the result to V_res
}
