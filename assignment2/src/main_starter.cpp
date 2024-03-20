#include <igl/readOFF.h>
#include <igl/writeOFF.h>
#include <imgui.h>
/*** insert any necessary libigl headers here ***/
#include <igl/per_face_normals.h>
#include <igl/copyleft/marching_cubes.h>
#include "viewer_proxy.h"
#include "uniform_grid.h"

using namespace std;

using namespace Eigen;

using Viewer = ViewerProxy;

// Input: imported points, #P x3
Eigen::MatrixXd P;

// Input: imported normals, #P x3
Eigen::MatrixXd N;

// Normals evaluated via PCA method, #P x3
Eigen::MatrixXd NP;

// Intermediate result: constrained points, #C x3
Eigen::MatrixXd constrained_points;

// Intermediate result: implicit function values at constrained points, #C x1
Eigen::VectorXd constrained_values;

// Parameter: degree of the polynomial
int polyDegree = 0;

// Parameter: Wendland weight function radius (make this relative to the size of the mesh)
double wendlandRadius = 0.1;

// Parameter: grid resolution
int grid_resolution_X = 20;
int grid_resolution_Y = 20;
int grid_resolution_Z = 20;

double grid_expansion = 1.05;

// Intermediate result: grid points, at which the imlicit function will be evaluated, #G x3
Eigen::MatrixXd grid_points;

// Intermediate result: implicit function values at the grid points, #G x1
Eigen::VectorXd grid_values;

// Intermediate result: grid point colors, for display, #G x3
Eigen::MatrixXd grid_colors;

// Intermediate result: grid lines, for display, #L x6 (each row contains
// starting and ending point of line segment)
Eigen::MatrixXd grid_lines;

// Output: vertex array, #V x3
Eigen::MatrixXd V;

// Output: face array, #F x3
Eigen::MatrixXi F;

// Output: face normals of the reconstructed mesh, #F x3
Eigen::MatrixXd FN;

Eigen::MatrixXd poly_values;

bool spatial_index_switch;

// Functions
void createGrid();
void evaluateImplicitFunc();
void evaluateImplicitFunc_PolygonSoup();
void getLines();
void pcaNormal();
bool callback_key_down(Viewer &viewer, unsigned char key, int modifiers);
bool isClosest(RowVector3d& point, int index);
RowVectorXd getPolyValue(const RowVector3d& point);
double getImplicitFuncVal(const vector<int>& possible_point_index, const RowVector3d& point);
double getWendLandFuncVal(const RowVector3d& point, const RowVector3d& constrained_point);
void normalize();
MatrixXd getCovarianceMatrix(const MatrixXd& X);

// Creates a grid_points array for the simple sphere example. The points are
// stacked into a single matrix, ordered first in the x, then in the y and
// then in the z direction. If you find it necessary, replace this with your own
// function for creating the grid.
void createGrid()
{
    grid_points.resize(0, 3);
    grid_colors.resize(0, 3);
    grid_lines.resize(0, 6);
    grid_values.resize(0);
    V.resize(0, 3);
    F.resize(0, 3);
    FN.resize(0, 3);

    // Grid bounds: axis-aligned bounding box
    Eigen::RowVector3d bb_min, bb_max;
    bb_min = P.colwise().minCoeff();
    bb_max = P.colwise().maxCoeff();
    
    //expansion the corner
    bb_min *= grid_expansion;
    bb_max *= grid_expansion;

    // Bounding box dimensions
    Eigen::RowVector3d dim = bb_max - bb_min;

    // Grid spacing
    const double dx = dim[0] / (double)(grid_resolution_X - 1);
    const double dy = dim[1] / (double)(grid_resolution_Y - 1);
    const double dz = dim[2] / (double)(grid_resolution_Z - 1);
    // 3D positions of the grid points -- see slides or marching_cubes.h for ordering
    grid_points.resize(grid_resolution_X * grid_resolution_Y * grid_resolution_Z, 3);
    // Create each gridpoint
    for (unsigned int x = 0; x < grid_resolution_X; ++x)
    {
        for (unsigned int y = 0; y < grid_resolution_Y; ++y)
        {
            for (unsigned int z = 0; z < grid_resolution_Z; ++z)
            {
                // Linear index of the point at (x,y,z)
                int index = x + grid_resolution_X * (y + grid_resolution_Y * z);
                // 3D point at (x,y,z)
                grid_points.row(index) = bb_min + Eigen::RowVector3d(x * dx, y * dy, z * dz);
            }
        }
    }
}

// Function for explicitly evaluating the implicit function for a sphere of
// radius r centered at c : f(p) = ||p-c|| - r, where p = (x,y,z).
// This will NOT produce valid results for any mesh other than the given
// sphere.
// Replace this with your own function for evaluating the implicit function
// values at the grid points using MLS
void evaluateImplicitFunc()
{
    //cal poly_values via degree
    for(unsigned i = 0; i < constrained_points.rows(); ++i) {
        RowVector3d point = constrained_points.row(i);
        RowVectorXd poly_value = getPolyValue(point);
        //initialization
        if (i == 0) {
            poly_values.resize(constrained_points.rows(), poly_value.cols());
        }
        poly_values.row(i) = poly_value;
    }
    
    // Scalar values of the grid points (the implicit function values)
    grid_values.resize(grid_resolution_X * grid_resolution_Y * grid_resolution_Z);
    UniformGrid uniform_grid(constrained_points, wendlandRadius);
    // Evaluate sphere's signed distance function at each gridpoint.
    for (unsigned int x = 0; x < grid_resolution_X; ++x)
    {
        for (unsigned int y = 0; y < grid_resolution_Y; ++y)
        {
            for (unsigned int z = 0; z < grid_resolution_Z; ++z)
            {
                // Linear index of the point at (x,y,z)
                int index = x + grid_resolution_X * (y + grid_resolution_Y * z);

                vector<int> possible_point_index = uniform_grid.getNeighbors(grid_points.row(index), wendlandRadius);
                // Value at (x,y,z) = implicit function for the sphere
                if (possible_point_index.size() != 0) {
                    grid_values[index] = getImplicitFuncVal(possible_point_index, grid_points.row(index));
                } else {
                    grid_values[index] = INFINITY;
                }
            }
        }
    }
}

double getImplicitFuncVal(const vector<int>& possible_point_index, const RowVector3d& point) {
    int n = possible_point_index.size();
    RowVectorXd weight(n);
    MatrixXd valid_poly_values(n, poly_values.cols());
    VectorXd vals(n);
    int cnt = 0;
    //build matrix
    for (int i : possible_point_index) {
        RowVector3d constrained_point = constrained_points.row(i);
        weight[cnt] = getWendLandFuncVal(point, constrained_point); 
        valid_poly_values.row(cnt) = poly_values.row(i);
        vals[cnt] = constrained_values[i];
        cnt++; 
    }
    MatrixXd weightMat = weight.asDiagonal();
    VectorXd c = (weightMat * valid_poly_values).bdcSvd(ComputeThinU | ComputeThinV).solve(weightMat * vals);
    RowVectorXd poly_value = getPolyValue(point);
    double implicit_func_val = poly_value * c;
    return implicit_func_val;
}

double getImplicitFuncVal_normal(const vector<int>& possible_point_index, const RowVector3d& point) {
    int n = possible_point_index.size();
    RowVectorXd weight(n);
    MatrixXd valid_poly_values(n, poly_values.cols());
    VectorXd vals(n);
    int cnt = 0;
    for (int i : possible_point_index) {
        RowVector3d neighbor_point = P.row(i);
        weight[cnt] = getWendLandFuncVal(point, neighbor_point); 
        valid_poly_values.row(cnt) = poly_values.row(i);
        vals[cnt] = (point - neighbor_point) * N.row(i).transpose();
        cnt++; 
    }
    MatrixXd weightMat = weight.asDiagonal();
    VectorXd c = (weightMat * valid_poly_values).bdcSvd(ComputeThinU | ComputeThinV).solve(weightMat * vals);
    RowVectorXd poly_value = getPolyValue(point);
    double implicit_func_val = poly_value * c;
    return implicit_func_val;
}

double getWendLandFuncVal(const RowVector3d& point, const RowVector3d& constrained_point) {
    double r = (point - constrained_point).norm();
    double ratio = r / wendlandRadius;
    return pow((1 - ratio), 4) * (4 * ratio + 1);
}

Eigen::RowVectorXd getPolyValue(const Eigen::RowVector3d& point) {
    int maxTerm = (1 + polyDegree) * (2 + polyDegree) * (3 + polyDegree) / 6;
    RowVectorXd poly_value(maxTerm);
    int loc = 0;
    double x = point[0];
    double y = point[1];
    double z = point[2];
    VectorXd power_x(polyDegree + 1), power_y(polyDegree + 1), power_z(polyDegree + 1);
    power_x[0] = power_y[0] = power_z[0] = 1.0;
    
    for(unsigned i = 1; i < polyDegree + 1; ++i) {
        power_x[i] = power_x[i - 1] * x;
        power_y[i] = power_y[i - 1] * y;
        power_z[i] = power_z[i - 1] * z;
    }
    
    for(unsigned i = 0; i < polyDegree + 1; ++i) {
        for(unsigned j = 0; i + j < polyDegree + 1; ++j) {
            for(unsigned k = 0; i + j + k < polyDegree + 1; ++k) {
                poly_value[loc++] = power_x[i] * power_y[j] * power_z[k];
            }
        }
    }
    return poly_value;
}

void evaluateImplicitFunc_PolygonSoup()
{
    // Replace with your code here, for "key == '5'"
    //only consider the original points P
    //cal poly_values via degree
    for(unsigned i = 0; i < P.rows(); ++i) {
        RowVector3d point = P.row(i);
        RowVectorXd poly_value = getPolyValue(point);
        //initialization
        if (i == 0) {
            poly_values.resize(P.rows(), poly_value.cols());
        }
        poly_values.row(i) = poly_value;
    }
    
    // Scalar values of the grid points (the implicit function values)
    grid_values.resize(grid_resolution_X * grid_resolution_Y * grid_resolution_Z);
    UniformGrid uniform_grid(P, wendlandRadius);
    // Evaluate sphere's signed distance function at each gridpoint.
    for (unsigned int x = 0; x < grid_resolution_X; ++x)
    {
        for (unsigned int y = 0; y < grid_resolution_Y; ++y)
        {
            for (unsigned int z = 0; z < grid_resolution_Z; ++z)
            {
                // Linear index of the point at (x,y,z)
                int index = x + grid_resolution_X * (y + grid_resolution_Y * z);

                vector<int> possible_point_index = uniform_grid.getNeighbors(grid_points.row(index), wendlandRadius);
                // Value at (x,y,z) = implicit function for the sphere
                if (possible_point_index.size() != 0) {
                    grid_values[index] = getImplicitFuncVal_normal(possible_point_index, grid_points.row(index));
                } else {
                    grid_values[index] = INFINITY;
                }
            }
        }
    }

}

// Code to display the grid lines given a grid structure of the given form.
// Assumes grid_points have been correctly assigned
// Replace with your own code for displaying lines if need be.
void getLines()
{
    int nnodes = grid_points.rows();
    grid_lines.resize(3 * nnodes, 6);
    int numLines = 0;

    for (unsigned int x = 0; x < grid_resolution_X; ++x)
    {
        for (unsigned int y = 0; y < grid_resolution_Y; ++y)
        {
            for (unsigned int z = 0; z < grid_resolution_Z; ++z)
            {
                int index = x + grid_resolution_X * (y + grid_resolution_Y * z);
                if (x < grid_resolution_X - 1)
                {
                    int index1 = (x + 1) + y * grid_resolution_X + z * grid_resolution_X * grid_resolution_Y;
                    grid_lines.row(numLines++) << grid_points.row(index), grid_points.row(index1);
                }
                if (y < grid_resolution_Y - 1)
                {
                    int index1 = x + (y + 1) * grid_resolution_X + z * grid_resolution_X * grid_resolution_Y;
                    grid_lines.row(numLines++) << grid_points.row(index), grid_points.row(index1);
                }
                if (z < grid_resolution_Z - 1)
                {
                    int index1 = x + y * grid_resolution_X + (z + 1) * grid_resolution_X * grid_resolution_Y;
                    grid_lines.row(numLines++) << grid_points.row(index), grid_points.row(index1);
                }
            }
        }
    }

    grid_lines.conservativeResize(numLines, Eigen::NoChange);
}

// Estimation of the normals via PCA.
void pcaNormal()
{   
    UniformGrid uniform_grid(P, wendlandRadius);
    NP.resize(P.rows(), 3);
    //estimate normal for every point
    for(unsigned i = 0; i < P.rows(); ++i) {
        RowVector3d point = P.row(i);
        vector<int> possible_point_index = uniform_grid.getNeighbors(point, wendlandRadius);
        MatrixXd neighbors(possible_point_index.size(), 3);
        int cnt = 0;
        for (auto i : possible_point_index) {
            neighbors.row(cnt++) = P.row(i);
        }
        MatrixXd covariance = getCovarianceMatrix(neighbors);
        SelfAdjointEigenSolver<MatrixXd> eigen(covariance);
        MatrixXd eigenvectors = eigen.eigenvectors();
        //vector with least lambda
        RowVector3d normal = eigenvectors.col(0);
        RowVector3d provided_normal = N.row(i);
        //judge orientation
        if (normal.dot(provided_normal) < 0) {
            normal = -normal;
        }
        NP.row(i) = normal;
    }
}

MatrixXd getCovarianceMatrix(const MatrixXd& X) {
    return 1 / (X.rows() - 1.0) * X.transpose() * X;
}

bool callback_key_down(Viewer &viewer, unsigned char key, int modifiers)
{
    if (key == '1')
    {
        // Show imported points
        viewer.data().clear();
        viewer.core().align_camera_center(P);
        viewer.data().point_size = 11;
        viewer.data().add_points(P, Eigen::RowVector3d(0, 0, 0));
    }

    if (key == '2')
    {
        // Show all constraints
        viewer.data().clear();
        viewer.core().align_camera_center(P);
        // Add your code for computing auxiliary constraint points here
        // Add code for displaying all points, as above
        //calculate bounding_box_diagonal length
        int n = P.rows();
        RowVector3d p_min = P.colwise().minCoeff();
        RowVector3d p_max = P.colwise().maxCoeff();
        double bounding_box_diagonal = (p_max - p_min).norm();
        constrained_points.resize(3 * n, 3);
        constrained_values.resize(3 * n, 1);

        for(unsigned i = 0; i < P.rows(); ++i) {
            constrained_points.row(i) = P.row(i);
            constrained_values.row(i) << 0;
        }
        
        double eps = 0.01 * bounding_box_diagonal;
        UniformGrid uniform_grid(P, eps);
        // add constrained points
        for (int i = 0; i < n; i++) {
            //find outside constrained point
            RowVector3d P_i_N = P.row(i) + eps * N.row(i).normalized();
            if (spatial_index_switch) {
                while (!uniform_grid.isClosest(P_i_N, i)) {
                    eps *= 0.5;
                    P_i_N = P.row(i) + eps * N.row(i).normalized();
                }
            } else {
                while (!isClosest(P_i_N, i)) {
                    eps *= 0.5;
                    P_i_N = P.row(i) + eps * N.row(i).normalized();
                }
            }
            constrained_points.row(i + n) = P_i_N;
            constrained_values.row(i + n) << eps;
            eps = 0.01 * bounding_box_diagonal;

            //find outside constrained point
            RowVector3d P_i_2N = P.row(i) - eps * N.row(i).normalized();
            if (spatial_index_switch) {
                while (!uniform_grid.isClosest(P_i_2N, i)) {
                    eps *= 0.5;
                    P_i_2N = P.row(i) - eps * N.row(i).normalized();
                }
            } else {
                while (!isClosest(P_i_2N, i)) {
                    eps *= 0.5;
                    P_i_2N = P.row(i) - eps * N.row(i).normalized();
                }
            }
            constrained_points.row(i + 2 * n) = P_i_2N;
            constrained_values.row(i + 2 * n) << -eps;
            eps = 0.01 * bounding_box_diagonal;
        }
        
        // Blue for original points
        viewer.data().point_size = 11;
        viewer.data().add_points(constrained_points.block(0, 0, n, 3), Eigen::RowVector3d(0, 0, 1));
        // Red for outside points
        viewer.data().add_points(constrained_points.block(n, 0, n, 3), Eigen::RowVector3d(1, 0, 0));
        // Green for inside points
        viewer.data().add_points(constrained_points.block(2 * n, 0, n, 3), Eigen::RowVector3d(0, 1, 0));
    }

    if (key == '3')
    {
        // Show grid points with colored nodes and connected with lines
        viewer.data().clear();
        viewer.core().align_camera_center(P);
        // Add code for creating a grid
        // Add your code for evaluating the implicit function at the grid points
        // Add code for displaying points and lines
        // You can use the following example:

        /*** begin: sphere example, replace (at least partially) with your code ***/
        // Make grid
        createGrid();

        // Evaluate implicit function
        evaluateImplicitFunc();

        // get grid lines
        getLines();

        // Code for coloring and displaying the grid points and lines
        // Assumes that grid_values and grid_points have been correctly assigned.
        grid_colors.setZero(grid_points.rows(), 3);

        // Build color map
        for (int i = 0; i < grid_points.rows(); ++i)
        {
            double value = grid_values(i);
            if (value < 0)
            {
                grid_colors(i, 1) = 1;
            }
            else
            {
                if (value > 0)
                    grid_colors(i, 0) = 1;
            }
        }

        // Draw lines and points
        viewer.data().point_size = 8;
        viewer.data().add_points(grid_points, grid_colors);
        viewer.data().add_edges(grid_lines.block(0, 0, grid_lines.rows(), 3),
                                grid_lines.block(0, 3, grid_lines.rows(), 3),
                                Eigen::RowVector3d(0.8, 0.8, 0.8));
    }

    if (key == '4')
    {
        // Show reconstructed mesh
        viewer.data().clear();
        // Code for computing the mesh (V,F) from grid_points and grid_values
        if ((grid_points.rows() == 0) || (grid_values.rows() == 0))
        {
            cerr << "Not enough data for Marching Cubes !" << endl;
            return true;
        }
        // Run marching cubes
        igl::copyleft::marching_cubes(grid_values, grid_points, grid_resolution_X, grid_resolution_Y, grid_resolution_Z, V, F);
        if (V.rows() == 0)
        {
            cerr << "Marching Cubes failed!" << endl;
            return true;
        }

        igl::per_face_normals(V, F, FN);
        viewer.data().set_mesh(V, F);
        viewer.data().show_lines = true;
        viewer.data().show_faces = true;
        viewer.data().set_normals(FN);

        igl::writeOFF("../data/mesh.off", V, F);
    }

    if (key == '5')
    {
        // Use the structure for key=='3' but replace the function evaluateImplicitFunc();
        // with a function performing the approximation of the implicit surface from polygon soup
        // Ref: Chen Shen, James F. O’Brien, and Jonathan Richard Shewchuk. Interpolating and approximating implicit surfaces from polygon soup.

        // Show grid points with colored nodes and connected with lines
        viewer.data().clear();
        viewer.core().align_camera_center(P);

        // Make grid
        createGrid();

        // Evaluate implicit function --> Function to be modified here
        evaluateImplicitFunc_PolygonSoup();

        // get grid lines
        getLines();

        // Display the reconstruction
        callback_key_down(viewer, '4', modifiers);
    }

    if (key == '6' || key == '7' || key == '8')
    {
        // Implement PCA Normal Estimation --> Function to be modified here
        pcaNormal();

        // To use the normals estimated via PCA instead of the input normals and then restaurate the input normals
        Eigen::MatrixXd N_tmp = N;
        N = NP;

        switch (key)
        {
        case '6':
            callback_key_down(viewer, '2', modifiers);
            break;
        case '7':
            callback_key_down(viewer, '3', modifiers);
            break;
        case '8':
            callback_key_down(viewer, '3', modifiers);
            callback_key_down(viewer, '4', modifiers);
            break;
        default:
            break;
        }

        // Restore input normals
        N = N_tmp;
    }

    return true;
}

bool callback_load_mesh(Viewer &viewer, string filename)
{
    igl::readOFF(filename, P, F, N);
    callback_key_down(viewer, '1', 0);
    return true;
}

bool isClosest(Eigen::RowVector3d& point, int index) {
    double min_dist = (P.row(index) - point).squaredNorm();
    for (int i = 0; i < P.rows(); i++) {
        if (i != index) {
            double dist = (P.row(i) - point).squaredNorm();
            if (dist <= min_dist) {
                return false;
            }
        }
    }
    return true;
}

void normalize() {
    //shift point cloud to origin
    RowVector3d mean = P.colwise().mean();

    //N x 3
    P = P.rowwise() - mean;

    //cal the eigenvectors
    //covariance matrix 3x3
    MatrixXd covariance = getCovarianceMatrix(P);
    
    SelfAdjointEigenSolver<MatrixXd> eigen(covariance);
    //sorted eigenvector
    MatrixXd eigenvectors = eigen.eigenvectors().rowwise().reverse();;
    //rotate point cloud to align the axis
    P = P * eigenvectors;
    N = N * eigenvectors;
    
    //scaling the point cloud according to diagonal
    RowVector3d p_min = P.colwise().minCoeff();
    RowVector3d p_max = P.colwise().maxCoeff();
    double bounding_box_diagonal = (p_max - p_min).norm();
    P = P / (bounding_box_diagonal);
}

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        cout << "Usage ex2_bin <mesh.off>" << endl;
        igl::readOFF("../data/sphere.off", P, F, N);
    }
    else
    {
        // Read points and normals
        igl::readOFF(argv[1], P, F, N);
    }

    Viewer& viewer = Viewer::get_instance();
    Viewer::Menu& menu = viewer.menu();

    viewer.callback_key_down = callback_key_down;

    menu.callback_draw_viewer_menu = [&]()
    {
        // Draw parent menu content
        menu.draw_viewer_menu();

        // Add new group
        if (ImGui::CollapsingHeader("Reconstruction Options", ImGuiTreeNodeFlags_DefaultOpen))
        {
            // Expose variable directly ...
            if (ImGui::Button("Reset Grid", ImVec2(-1, 0)))
            {
                std::cout << "ResetGrid\n";
                // Recreate the grid
                createGrid();
                // Switch view to show the grid
                callback_key_down(viewer, '3', 0);
            }

            // TODO: Add more parameters to tweak here...
            
            if(ImGui::Button("Spatial Index Speedup")) {
                spatial_index_switch = !spatial_index_switch;
                cout << "spatial_index_switch:" << boolalpha << spatial_index_switch << endl;
            }
            ImGui::InputDouble("grid_expansion", &grid_expansion, 0, 0);
            ImGui::InputDouble("wendlandRadius", &wendlandRadius, 0, 0);
            ImGui::InputInt("polyDegree", &polyDegree, 0, 0);
            ImGui::InputInt("grid_resolution_X", &grid_resolution_X, 0, 0);
            ImGui::InputInt("grid_resolution_Y", &grid_resolution_Y, 0, 0);
            ImGui::InputInt("grid_resolution_Z", &grid_resolution_Z, 0, 0);

        }
    };

    cout << "normalizing cloud point...." << endl;
    normalize();
    cout << "finished normalizing" << endl;
    callback_key_down(viewer, '1', 0);

    viewer.launch();
}

