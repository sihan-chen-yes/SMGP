#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui.h>
#include <igl/slice_into.h>
#include <igl/rotate_by_quat.h>
#include <igl/readTGF.h>
#include <igl/readDMAT.h>
#include <igl/forward_kinematics.h>
#include <igl/directed_edge_parents.h>
#include <igl/cotmatrix.h>
#include <igl/invert_diag.h>
#include <igl/dqs.h>
#include <igl/grad.h>
#include <igl/cat.h>
#include <igl/per_face_normals.h>

#include <igl/slice.h>

//activate this for alternate UI (easier to debug but no interactive updates, turn this OFF for your report)
//#define UPDATE_ONLY_ON_UP

using namespace std;
using namespace Eigen;
using namespace igl;
using Viewer = igl::opengl::glfw::Viewer;

Viewer viewer;
/**
 * below contains lots of switches and confusing and tedious code for acceleration,
 * so that we can get a comparatively smooth animation,
 * one could ignore them and grasp the main logistic for first glance! which's commented as acceleration
 */
//vertex array, #V x3
Eigen::MatrixXd V(0, 3), V_original(0, 3);
//face array, #F x3
Eigen::MatrixXi F(0, 3);
// for skeleton
// joints
Eigen::MatrixXd C(0, 3), C_original(0, 3);
// bones connecting joints
Eigen::MatrixXi BE(0, 2);
//parent bones index
Eigen::VectorXi P;
//handle_id per vertex
Eigen::VectorXi handle_id(0, 1);
// pose target
std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> q_final_list;

// pose per frame
std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> q_list;
// max max_frames for sequence mode
int max_frames = 0;
// global transformation #BEx4 x 3
//Eigen::MatrixXd T;
std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> vQ;
std::vector<Eigen::Vector3d> vT;

//current frame starting from 0
int current_frames = 0;
//false for normal order true for reverse order
bool animation_order_reverse = false;
//number of max_frames to interpolate
int interpolation_frames = 60;

//skinning weight per vertex #BE x #V
MatrixXd vertex_skinning_weight;
//skinning weight per face #BE x #F
MatrixXd face_skinning_weight;
//choose coloring bone
int coloring_bone_id = 0;
//whether to choose handles mamually
bool select_handles = false;
bool need_to_compute_skinning_weight = true;
//selected handles or not
bool selected = false;
//for acceleration
bool prefactored = false;
SparseMatrix<double> Hff, Hfc, G;
//constrained and free vertex index
VectorXi constrain_points;
VectorXi free_points;
//constrained vertex position
MatrixXd V_c;
//prefactoring
Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>, Eigen::RowMajor> solver;
//unpose operation
MatrixXd V_EG_pose(0, 3);
MatrixXd V_EG_unposed(0, 3);
//container for all quaternion
std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> q_EG_list;
//container for quaternion and translation of each example
vector<vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>> egQs;
vector<vector<Eigen::Vector3d>> egTs;
//switch for acceleration
bool vertex_unposed = false;
bool face_unposed = false;
bool RBF_interpolated = false;
//coeff to calculate aj
MatrixXd c;
MatrixXd unpose_displacement(0, 3);
int num_eg = 0;
//choosing vertex_unposed shape to present
int unposed_shape = 0;
//T 'displacement' of face in rest pose
vector<vector<MatrixXd>> deformation_gradient;
//acceleration
vector<Matrix3d> Q_j_f_list;
vector<Matrix3d> S_j_f_list;
//decomposition switch
bool decompose_T = false;

enum AnimationMode {
    SEQUENCE, INTERPOLATION, IDENTIY
};

enum VisualizeMode {
    ANIMATION, SELECTION, SKINNING_WEIGHT, UNPOSED
};

enum BSMode {
    LBS, DQS, FBS, VCAD, FCAD
};

AnimationMode animation_mode = IDENTIY;
VisualizeMode visualization_mode = ANIMATION;
BSMode BS_mode = LBS;

bool callback_key_down(Viewer &viewer, unsigned char key, int modifiers);

bool skeleton_animation_pre_draw(Viewer &viewer);

void update_current_frames(int max_frames);

void skinning_weight_computation();

void handle_selection(int bone_id);

void get_skinning_weight();

void transform_bone_vertex();

void get_dQ_current(std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> &dQ);

void linear_blend_skinning(const MatrixXd &V_original, const MatrixXd &vertex_skinning_weight, const vector<Eigen::Quaterniond,
        Eigen::aligned_allocator<Eigen::Quaterniond>> &vQ, const vector<Eigen::Vector3d> &vT, MatrixXd &V);

void dual_quaternion_skinning(const MatrixXd &V_original, const MatrixXd &vertex_skinning_weight, const vector<Eigen::Quaterniond,
        Eigen::aligned_allocator<Eigen::Quaterniond>> &vQ, const vector<Eigen::Vector3d> &vT, MatrixXd &V);

void face_blend_skinning(const MatrixXd &V_original, const MatrixXd &face_skinning_weight, const vector<Eigen::Quaterniond,
        Eigen::aligned_allocator<Eigen::Quaterniond>> &vQ, MatrixXd &V);

void vertex_context_aware_deformation(const MatrixXd &V_original, const MatrixXd &vertex_skinning_weight, const vector <Eigen::Quaterniond,
Eigen::aligned_allocator<Eigen::Quaterniond>> &vQ, const vector <Eigen::Vector3d> &vT, MatrixXd &V);

double get_RBF_distance(const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> &Q0,
                        const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> &Q1,
                        const vector<Eigen::Vector3d> &T0,
                        const vector<Eigen::Vector3d> &T1);

MatrixXd get_fusing_transformation(int j, const MatrixXd &vertex_skinning_weight, const vector<Eigen::Quaterniond,
        Eigen::aligned_allocator<Eigen::Quaterniond>> &vQ, const vector<Eigen::Vector3d> &vT);

void fill_egQs();

VectorXd get_a();

vector <MatrixXd> get_avg_R_per_face(const MatrixXd &face_skinning_weight, const vector <Eigen::Quaterniond,
Eigen::aligned_allocator<Eigen::Quaterniond>> &vQ);

void possion_stitching(bool use_prefactored, const MatrixXd &V_old, const MatrixXd &B, MatrixXd &V_new);

void face_context_aware_deformation(const MatrixXd &V_original, const MatrixXd &face_skinning_weight,
                                    const vector<Eigen::Quaterniond,
                                            Eigen::aligned_allocator<Eigen::Quaterniond>> &vQ, MatrixXd &V);

void decomposition(const MatrixXd &T, Matrix3d &R, Matrix3d &S);

MatrixXd get_avg_rotation(const VectorXd &weight, const vector<Eigen::Quaterniond,
        Eigen::aligned_allocator<Eigen::Quaterniond>> &vQ);

bool load_files(string base_path) {
    /*
     * loading files with errors captures
     */
    string mesh_filename = base_path + "/rest.obj";
    igl::read_triangle_mesh(mesh_filename, V, F);
    cout << "loaded mesh" << endl;
    viewer.data().clear();
    viewer.data().set_mesh(V, F);

    viewer.core().align_camera_center(V);
    V_original = V;
    handle_id.setConstant(V.rows(), 1, -1);

    string skeleton_filename = base_path + "/skeleton.tgf";
    ifstream skeleton_file(skeleton_filename);
    if (skeleton_file.good()) {
        igl::readTGF(skeleton_filename, C, BE);
        C_original = C;
        igl::directed_edge_parents(BE, P);
        cout << "loaded skeleton" << endl;
    } else {
        cout << "load failed!" << endl;
    }

    string pose_filename = base_path + "/pose.dmat";
    ifstream pose_file(pose_filename);
    if (pose_file.good()) {
        MatrixXd Q;
        igl::readDMAT(pose_filename, Q);
        max_frames = Q.rows() / BE.rows();
        for (int i = 0; i < max_frames; ++i) {
            for (int j = 0; j < BE.rows(); ++j) {
                RowVector4d r = Q.row(i * BE.rows() + j);
                Quaterniond q(r(3), r(0), r(1), r(2));
                q_list.push_back(q);
            }
        }
        cout << "loaded pose" << endl;
    }

    string pose_target_filename = base_path + "/pose_target.dmat";
    ifstream pose_target_file(pose_target_filename);
    if (pose_target_file.good()) {
        MatrixXd Q_final;
        igl::readDMAT(pose_target_filename, Q_final);

        for (int i = 0; i < BE.rows(); ++i) {
            RowVector4d r = Q_final.row(i);
            Quaterniond q(r(3), r(0), r(1), r(2));
            q_final_list.push_back(q);
        }
        cout << "loaded pose_target" << endl;
    }

    string handles_filename = base_path + "/handles.dmat";
    ifstream handles_file(handles_filename);
    if (handles_file.good()) {
        igl::readDMAT(handles_filename, handle_id);
        cout << "loaded handles" << endl;
    }

    //assuming eg0~3
    while (true) {
        string EG_dmat_filename = base_path + "/eg" + to_string(num_eg) + ".dmat";
        ifstream EG_dmat_file(EG_dmat_filename);
        if (EG_dmat_file.good()) {
            MatrixXd Q;
            igl::readDMAT(EG_dmat_filename, Q);
            for (int i = 0; i < BE.rows(); ++i) {
                RowVector4d r = Q.row(i);
                Quaterniond q(r(3), r(0), r(1), r(2));
                q_EG_list.push_back(q);
            }
            cout << "loaded " + EG_dmat_filename << endl;
        } else {
            break;
        }
        string EG_obj_filename = base_path + "/eg" + to_string(num_eg) + ".obj";
        ifstream EG_obj_file(EG_obj_filename);
        if (EG_obj_file.good()) {
            MatrixXd V_EG, F_EG;
            igl::read_triangle_mesh(EG_obj_filename, V_EG, F_EG);
            MatrixXd V_EG_pose_new;
            igl::cat(1, V_EG_pose, V_EG, V_EG_pose_new);
            V_EG_pose = V_EG_pose_new;
            cout << "loaded " + EG_obj_filename << endl;
        } else {
            break;
        }
        num_eg++;
    }

    return true;
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        cout << "Usage assignment6 loading files" << endl;
        string base_path = "../data/context-aware/";
        load_files(base_path);
    } else {
        load_files(argv[1]);
    }


    igl::opengl::glfw::imgui::ImGuiPlugin plugin;
    viewer.plugins.push_back(&plugin);
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    plugin.widgets.push_back(&menu);

    menu.callback_draw_viewer_menu = [&]() {
        // Draw parent menu content
        menu.draw_viewer_menu();

        // Add new group
        if (ImGui::CollapsingHeader("BS Controls", ImGuiTreeNodeFlags_DefaultOpen)) {
            int animation_mode_type = static_cast<int>(animation_mode);
            int visualization_mode_type = static_cast<int>(visualization_mode);
            int BS_mode_type = static_cast<int>(BS_mode);

            if (ImGui::Combo("Animation Mode", &animation_mode_type, "SEQUENCE\0INTERPOLATION\0IDENTITY\0")) {
                animation_mode = static_cast<AnimationMode>(animation_mode_type);
                current_frames = 0;
            }

            if (ImGui::Combo("Visualize Mode", &visualization_mode_type, "ANIMATION\0SELECTION\0SKINNING_WEIGHT\0UNPOSED\0")) {
                visualization_mode = static_cast<VisualizeMode>(visualization_mode_type);
            }

            if (ImGui::Combo("Blend Skinning Mode", &BS_mode_type, "LBS\0DQS\0FBS\0VCAD\0FCAD\0")) {
                BS_mode = static_cast<BSMode>(BS_mode_type);
            }

            ImGui::InputInt("interpolation_frames", &interpolation_frames, 0, 0);
            ImGui::InputInt("coloring_bone_id", &coloring_bone_id, 0, 0);
            ImGui::InputInt("unposed_shape", &unposed_shape, 0, 0);
            ImGui::Checkbox("select_handles", &select_handles);
            ImGui::Checkbox("decompose_T", &decompose_T);
        }
    };

    viewer.callback_key_down = callback_key_down;
    viewer.callback_pre_draw = skeleton_animation_pre_draw;

    viewer.data().point_size = 10;
    viewer.core().set_rotation_type(igl::opengl::ViewerCore::ROTATION_TYPE_TRACKBALL);
    viewer.launch();
}


bool skeleton_animation_pre_draw(Viewer &viewer) {
    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> dQ;
    get_dQ_current(dQ);
    if (dQ.size() == 0) {
        return false;
    }
    // given quaternion Q compute the global transformation T for joints
    igl::forward_kinematics(C_original, BE, P, dQ, vQ, vT);
    transform_bone_vertex();
    //only visualize skeleton
    viewer.data().clear();
    //color for skeleton
    Eigen::MatrixXd point_colors = Eigen::MatrixXd::Zero(C.rows(), 3);
    Eigen::MatrixXd edge_colors = Eigen::MatrixXd::Zero(BE.rows(), 3);
    //Red color for points
    point_colors.col(0) = Eigen::VectorXd::Ones(C.rows());
    //green color for edges
    edge_colors.col(1) = Eigen::VectorXd::Ones(BE.rows());

    //visualize skeleton
    viewer.data().set_points(C, point_colors);
    viewer.data().set_edges(C, BE, edge_colors);
    viewer.data().show_overlay_depth = false;

    return false;
}

bool skinning_weight_pre_draw(Viewer &viewer){
    //if need to generate handles automatically
    if (select_handles && !selected) {
        //initialization
        handle_id.setConstant(V.rows(), 1, -1);
        //geometric selection
        for (int i = 0; i < BE.rows(); ++i) {
            handle_selection(i);
        }
        selected = true;
        need_to_compute_skinning_weight = true;
    }

    //acceleration
    //get skinning weight
    if (need_to_compute_skinning_weight) {
        if (selected) {
            cout << "using generated handles" << endl;
        } else {
            cout << "using given handles" << endl;
        }
        skinning_weight_computation();
        need_to_compute_skinning_weight = false;
    }

    // initialize vertex colors
    MatrixXd vertex_colors(V.rows(), 3);
    //Blue
    Eigen::RowVector3d blue(0.0, 0.0, 1.0);
    //Yellow
    Eigen::RowVector3d yellow(1.0, 1.0, 0.0);
    if (visualization_mode == SELECTION) {
        //show the selection of handles
        for (int i = 0; i < V.rows(); ++i) {
            if (handle_id[i] == coloring_bone_id) {
                vertex_colors.row(i) = yellow;
            } else {
                vertex_colors.row(i) = blue;
            }
        }
    } else if (visualization_mode == SKINNING_WEIGHT) {
        //show the skinning_weight of vertex
        Eigen::VectorXd ones(V.rows());
        ones.setOnes();

        VectorXd w_k = vertex_skinning_weight.row(coloring_bone_id);
        vertex_colors = (ones - w_k) * blue + w_k * yellow;
    } else {
        //default color
        vertex_colors.col(0).setZero();
        vertex_colors.col(1).setZero();
        vertex_colors.col(2).setOnes();
    }

    Eigen::RowVector3d red(1.0, 0.0, 0.0);
    Eigen::RowVector3d green(0.0, 1.0, 0.0);
    int start = BE(coloring_bone_id, 0);
    int end = BE(coloring_bone_id, 1);
    Eigen::MatrixXd point_colors = Eigen::MatrixXd::Zero(C.rows(), 3);
    Eigen::MatrixXd edge_colors = Eigen::MatrixXd::Zero(BE.rows(), 3);

    //draw skeleton
    //Red color for points
    point_colors.col(0) = Eigen::VectorXd::Ones(C.rows());
    point_colors.row(start) = yellow;
    point_colors.row(end) = yellow;
    //green color for edges
    edge_colors.col(1) = Eigen::VectorXd::Ones(BE.rows());
    edge_colors.row(coloring_bone_id) = yellow;

    viewer.data().set_points(C_original, point_colors);
    viewer.data().set_edges(C_original, BE, edge_colors);
    viewer.data().show_overlay_depth = false;
    //update the vertex position all the time
    viewer.data().set_mesh(V_original, F);

    viewer.data().set_colors(vertex_colors);
    return false;
}


bool BS_pre_draw(Viewer &viewer) {
    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> dQ;

    get_dQ_current(dQ);
    if (dQ.size() == 0) {
        return false;
    }
    // given quaternion Q compute the global transformation T for joints
    igl::forward_kinematics(C_original, BE, P, dQ, vQ, vT);
    get_skinning_weight();

    //choose blend skinning methods
    if (BS_mode == LBS) {
        //LBS for vertex on mesh
        linear_blend_skinning(V_original, vertex_skinning_weight, vQ, vT, V);
    } else if (BS_mode == DQS) {
        dual_quaternion_skinning(V_original, vertex_skinning_weight, vQ, vT, V);
    } else if (BS_mode == FBS) {
        face_blend_skinning(V_original, face_skinning_weight, vQ, V);
    } else if (BS_mode == VCAD) {
        vertex_context_aware_deformation(V_original, vertex_skinning_weight, vQ, vT, V);
    } else if (BS_mode == FCAD) {
        face_context_aware_deformation(V_original, face_skinning_weight, vQ, V);
    }

    transform_bone_vertex();
    //Blue for vertex
    MatrixXd vertex_colors(V.rows(), 3);
    vertex_colors.setZero();
    vertex_colors.col(2) = Eigen::VectorXd::Ones(V.rows());

    viewer.data().clear();

    //color for skeleton
    Eigen::MatrixXd point_colors = Eigen::MatrixXd::Zero(C.rows(), 3);
    Eigen::MatrixXd edge_colors = Eigen::MatrixXd::Zero(BE.rows(), 3);
    //Red color for points
    point_colors.col(0) = Eigen::VectorXd::Ones(C.rows());
    //green color for edges
    edge_colors.col(1) = Eigen::VectorXd::Ones(BE.rows());

    //only visualize specific unposed shape
    if (visualization_mode == UNPOSED) {
        viewer.data().set_mesh(V, F);
        viewer.data().set_colors(vertex_colors);
    } else {
        //visualize skeleton
        viewer.data().set_points(C, point_colors);
        viewer.data().set_edges(C, BE, edge_colors);
        viewer.data().show_overlay_depth = false;

        viewer.data().set_mesh(V, F);
        viewer.data().set_colors(vertex_colors);
    }

    return false;
}

bool callback_key_down(Viewer &viewer, unsigned char key, int modifiers) {
    bool handled = false;

    //for 3.Skeleton Animation
    if (key == 'Q') {
        viewer.callback_pre_draw = skeleton_animation_pre_draw;
        handled = true;
    }

    //for 4.Harmonic Skinning Weight
    if (key == 'W') {
        viewer.callback_pre_draw = skinning_weight_pre_draw;
        handled = true;
    }

    //for 5.6.7.8.
    if (key == 'E') {
        viewer.callback_pre_draw = BS_pre_draw;
        handled = true;
    }

    return handled;
}

Quaterniond quaternion_interpolation(const Quaterniond &q0, const Quaterniond &q1, double t) {
    double dot_product = q0.dot(q1);
    //use shortest rotation path
    double theta = std::acos(abs(dot_product));
    double scalar0;
    double scalar1;
    //if very close use linear interpolation
    if (abs(theta) < 1e-10) {
        scalar0 = 1 - t;
        scalar1 = t;
    } else {
        //SLERP interpolation
        double sin_theta = std::sin(theta);
        scalar0 = std::sin((1 - t) * theta) / sin_theta;
        scalar1 = std::sin(t * theta) / sin_theta;
    }

    if(dot_product < 0) {
        scalar1 = -scalar1;
    }
    Quaterniond q;
    q.coeffs() = scalar0 * q0.coeffs() + scalar1 * q1.coeffs();
    return q;
}

//control video play
void update_current_frames(int max_frames) {
    // play the video from start to end, from end to start
    // frames for next step
    if (animation_order_reverse) {
        //reverse order
        current_frames -= 1;
        if (current_frames == -1) {
            current_frames = 0;
            animation_order_reverse = false;
        }
    } else {
        //normal order
        current_frames += 1;
        if (current_frames == max_frames) {
            current_frames = max_frames - 1;
            animation_order_reverse = true;
        }
    }
}

void skinning_weight_computation() {
    V = V_original;
    //number of bones
    //iterate bones and solve Lw_k = 0 like assignment5
    vertex_skinning_weight.resize(BE.rows(), V.rows());
    face_skinning_weight.resize(BE.rows(), F.rows());

    //cholesky decomposition to compute Aff Afc
    SparseMatrix<double> M, Aff, A, Lw, M_inv, L, Afc;
    igl::cotmatrix(V, F, Lw);
    igl::massmatrix(V, F, MASSMATRIX_TYPE_DEFAULT, M);
    igl::invert_diag(M, M_inv);

    L = M_inv * Lw;
    A = L.transpose() * L;
    VectorXi constrained_weight_skinning(V.rows());
    VectorXi free_weight_skinning(V.rows());
    int constrained_cnt;
    int free_cnt;
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>, Eigen::RowMajor> solver;
    VectorXd w_k_f, w_k_c, w_k;
    for (int k = 0; k < BE.rows(); ++k) {
        //prepare weight skinning linear system to solve
        w_k.resize(V.rows());
        w_k.setZero();

        assert(handle_id.rows() == V.rows());

        constrained_cnt = 0;
        free_cnt = 0;
        for (int i = 0; i < handle_id.rows(); ++i) {
            if (handle_id[i] == -1) {
                //free weight skinning
                free_weight_skinning[free_cnt++] = i;
            } else {
                //constrained weight skinning
                constrained_weight_skinning[constrained_cnt++] = i;
                w_k[i] = (handle_id[i] == k) ? 1 : 0;
            }
        }
        free_weight_skinning.conservativeResize(free_cnt);
        constrained_weight_skinning.conservativeResize(constrained_cnt);

        slice(A, free_weight_skinning, free_weight_skinning, Aff);
        slice(A, free_weight_skinning, constrained_weight_skinning, Afc);

        slice(w_k, free_weight_skinning,1, w_k_f);
        slice(w_k, constrained_weight_skinning, 1, w_k_c);

        solver.compute(Aff);
        w_k_f = solver.solve(-Afc * w_k_c);
        slice_into(w_k_f, free_weight_skinning, 1, w_k);
        //per vertex skinning weight
        vertex_skinning_weight.row(k) = w_k;

        //per face skinning weight
        for (int i = 0; i < F.rows(); ++i) {
            RowVector3i f = F.row(i);
            face_skinning_weight(k, i) = (w_k[f[0]] + w_k[f[1]] + w_k[f[2]]) / 3.0;
        }
    }

    //SANITY CHECK
    for (int i = 0; i < V.rows(); ++i) {
        assert(abs(vertex_skinning_weight.col(i).sum() - 1) <= 1e-6);
    }
    for (int i = 0; i < F.rows(); ++i) {
        assert(abs(face_skinning_weight.col(i).sum() - 1) <= 1e-6);
    }
}

//pick vertex around given bone automatically
void handle_selection(int bone_id) {
    V = V_original;
    // geometric selection
    //select points around bone bone_id
    RowVector2i bone_edge = BE.row(bone_id);
    int start = bone_edge(0);
    int end = bone_edge(1);
    RowVector3d v_start = C_original.row(start);
    RowVector3d v_end = C_original.row(end);
    //find the shortest distance around v_start and v_end point
    double min = INFINITY;
    double r;
    Vector3d b = (v_end - v_start).normalized();

    for (int i = 0; i < V.rows(); ++i) {
        RowVector3d v1 = V.row(i) - v_start;
        RowVector3d v2 = V.row(i) - v_end;
        RowVector3d p = v1.dot(b) * b;
        r = (v1 - p).norm();
        if (r < min && v1.dot(p) >= 0 && v2.dot(p) <= 0) {
            min = r;
        }
    }
    //performance is not so good for 1.5 I choose 2
    min *= 2;
    //find points closer to min
    for (int i = 0; i < V.rows(); ++i) {
        RowVector3d v1 = V.row(i) - v_start;
        RowVector3d v2 = V.row(i) - v_end;
        //projection vector
        RowVector3d p = v1.dot(b) * b;
        //inside cylinder
        if ((v1 - p).norm() <= min && v1.dot(p) >= 0 && v2.dot(p) <= 0) {
            handle_id[i] = bone_id;
        }
    }
}

//get quaternion of next frame and update frame
void get_dQ_current(std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> &dQ) {
    if (animation_mode == SEQUENCE) {
        //check
        if (q_list.size() == 0) {
            cout << "no pose loaded" << endl;
            return;
        }
        int start_index = current_frames * BE.rows();
        // pose for a frame
        for (int i = 0; i < BE.rows(); ++i) {
            Quaterniond q = q_list[start_index + i];
            dQ.push_back(q);
        }
        // play the video from start to end, from end to start
        // frames for next step
        update_current_frames(max_frames);
    } else if (animation_mode == INTERPOLATION){
        //check
        if (q_final_list.size() == 0) {
            cout << "no target pose loaded" << endl;
            return;
        }
        // interpolation between initial pose and target pose
        Quaterniond q0(1.0, 0.0, 0.0, 0.0);
        double t = ((double) current_frames) / interpolation_frames;
        assert(t >= 0 && t <= 1);
        assert(BE.rows() == q_final_list.size());
        for (int i = 0; i < BE.rows(); ++i) {
            Quaterniond q1 = q_final_list[i];
//            Quaterniond q_interp = q0.slerp(t, q1);
            Quaterniond q_interp = quaternion_interpolation(q0, q1, t);
            dQ.push_back(q_interp);
        }
        // play the video from start to end, from end to start
        // frames for next step
        update_current_frames(interpolation_frames);
    }
    return;
}

void get_skinning_weight() {
    //if need to generate handles
    if (select_handles && !selected) {
        //initialization
        handle_id.setConstant(V.rows(), 1, -1);
        //geometric selection
        for (int i = 0; i < BE.rows(); ++i) {
            handle_selection(i);
        }
        selected = true;
        need_to_compute_skinning_weight = true;
    }

    //acceleration
    //get skinning weight
    if (need_to_compute_skinning_weight) {
        if (selected) {
            cout << "using generated handles" << endl;
        } else {
            cout << "using given handles" << endl;
        }
        skinning_weight_computation();
        need_to_compute_skinning_weight = false;
    }
}

//rigid transformation
void transform_bone_vertex() {
    //iterate bones
    for (int i = 0; i < BE.rows(); ++i) {
        Quaterniond q = vQ[i];
        Vector3d t = vT[i];
        MatrixXd R = q.toRotationMatrix();
        int end_index = BE(i, 1);
        C.row(end_index) = (R * C_original.row(end_index).transpose() + t).transpose();
    }
}

void linear_blend_skinning(const MatrixXd &V_original, const MatrixXd &vertex_skinning_weight, const vector<Eigen::Quaterniond,
        Eigen::aligned_allocator<Eigen::Quaterniond>> &vQ, const vector<Eigen::Vector3d> &vT, MatrixXd &V) {
    for (int i = 0; i < V.rows(); ++i) {
        RowVector3d v;
        v.setZero();
        //LBS
        for (int k = 0; k < BE.rows(); ++k) {
            Quaterniond q = vQ[k];
            Vector3d t = vT[k];
            MatrixXd R = q.toRotationMatrix();
            v += vertex_skinning_weight(k, i) * (R * V_original.row(i).transpose() + t);
        }
        V.row(i) = v;
    }
}

void dual_quaternion_skinning(const MatrixXd &V_original, const MatrixXd &vertex_skinning_weight, const vector<Eigen::Quaterniond,
         Eigen::aligned_allocator<Eigen::Quaterniond>> &vQ, const vector<Eigen::Vector3d> &vT, MatrixXd &V) {
    vector<Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> veQ(vQ.size());
    // follow igl::dqs

    // vertex_skinning_weight:#BE x #V
    // Convert quats + trans into dual parts
    for (int k = 0; k < vertex_skinning_weight.rows(); ++k) {
        const Quaterniond &q = vQ[k];
        veQ[k].w() = -0.5 * (vT[k](0) * q.x() + vT[k](1) * q.y() + vT[k](2) * q.z());
        veQ[k].x() = 0.5 * (vT[k](0) * q.w() + vT[k](1) * q.z() - vT[k](2) * q.y());
        veQ[k].y() = 0.5 * (-vT[k](0) * q.z() + vT[k](1) * q.w() + vT[k](2) * q.x());
        veQ[k].z() = 0.5 * (vT[k](0) * q.y() - vT[k](1) * q.x() + vT[k](2) * q.w());
    }

    for(int i = 0;i < V_original.rows();i++) {
        Quaterniond b0(0,0,0,0);
        Quaterniond be(0,0,0,0);
        // Loop over handles
        for(int k = 0; k < vertex_skinning_weight.rows(); k++) {
            //check dual quaternion hemisphere w.r.t base quaternion q[0]
            if (vQ[0].coeffs().dot(vQ[k].coeffs()) +
                veQ[0].coeffs().dot(veQ[k].coeffs()) < 0) {
                b0.coeffs() += -1 * vertex_skinning_weight(k, i) * vQ[k].coeffs();
                be.coeffs() += -1 * vertex_skinning_weight(k, i) * veQ[k].coeffs();
            } else {
                b0.coeffs() += vertex_skinning_weight(k, i) * vQ[k].coeffs();
                be.coeffs() += vertex_skinning_weight(k, i) * veQ[k].coeffs();
            }
        }
        Quaterniond ce = be;
        ce.coeffs() /= b0.norm();
        Quaterniond c0 = b0;
        c0.coeffs() /= b0.norm();
        // See algorithm 1 in "Geometric skinning with approximate dual quaternion
        // blending" by Kavan et al
        RowVector3d v = V_original.row(i);
        RowVector3d d0 = c0.vec();
        RowVector3d de = ce.vec();
        Quaterniond::Scalar a0 = c0.w();
        Quaterniond::Scalar ae = ce.w();
        V.row(i) =  v + 2*d0.cross(d0.cross(v) + a0*v) + 2*(a0*de - ae*d0 + d0.cross(de));
    }
}

void face_blend_skinning(const MatrixXd &V_original, const MatrixXd &face_skinning_weight, const vector<Eigen::Quaterniond,
        Eigen::aligned_allocator<Eigen::Quaterniond>> &vQ, MatrixXd &V) {
    //for a specific frame
    int m = F.rows();
    //prepare rhs
    vector<MatrixXd> R_per_face = get_avg_R_per_face(face_skinning_weight, vQ);
    MatrixXd B(3 * m, 3);
    for (int i = 0; i < m; ++i) {
        MatrixXd R = R_per_face[i];

        B.row(i) = R.row(0);
        B.row(m + i) = R.row(1);
        B.row(2 * m + i) = R.row(2);
    }

    //acceleration
    //possion stitching
    //if not prefactored then just do prefactoring!
    bool using_prefactored;
    if (!prefactored) {
        using_prefactored = false;
        prefactored = true;
    } else {
        using_prefactored = true;
    }

    possion_stitching(using_prefactored, V_original, B, V);
}

void vertex_context_aware_deformation(const MatrixXd &V_original, const MatrixXd &vertex_skinning_weight, const vector<Eigen::Quaterniond,
Eigen::aligned_allocator<Eigen::Quaterniond>> &vQ, const vector<Eigen::Vector3d> &vT, MatrixXd &V) {
    //acceleration
    if (!vertex_unposed) {
        unpose_displacement.resize(V_EG_pose.rows(), 3);
        V_EG_unposed.resize(V_EG_pose.rows(), 3);
        fill_egQs();

        //unpose given example
        for (int i = 0; i < num_eg; ++i) {
            //find vertex_unposed position for point j
            MatrixXd V_pose = V_EG_pose.block(i * V.rows(), 0, V.rows(), 3);
            for (int j = 0; j < V_pose.rows(); ++j) {
                Vector3d v = V_pose.row(j);
                //fusing transformation
                MatrixXd T = get_fusing_transformation(j, vertex_skinning_weight, egQs[i], egTs[i]);

                RowVector3d v_unpose = (T.inverse() * v.homogeneous()).head<3>();
                V_EG_unposed.row(i * V.rows() + j) = v_unpose;
                unpose_displacement.row(i * V.rows() + j) = v_unpose - V_original.row(j);
            }
        }

        //fin unpose operation
        vertex_unposed = true;
    }

    //if only visualize unposed shape
    if (visualization_mode == UNPOSED) {
        if (unposed_shape >= num_eg) {
            cout <<"invalid vertex_unposed shape" << endl;
            return;
        }
        V = V_EG_unposed.block(unposed_shape * V.rows(), 0, V.rows(), 3);
        return;
    }

    VectorXd a = get_a();

    for (int i = 0; i < V.rows(); ++i) {
        MatrixXd T = get_fusing_transformation(i, vertex_skinning_weight, vQ, vT);
        VectorXd v = V_original.row(i);
        for (int j = 0; j < num_eg; ++j) {
            v += a[j] * unpose_displacement.row(j * V.rows() + i);
        }
        V.row(i) = (T * v.homogeneous()).head<3>();
    }
}

void face_context_aware_deformation(const MatrixXd &V_original, const MatrixXd &face_skinning_weight, const vector<Eigen::Quaterniond,
        Eigen::aligned_allocator<Eigen::Quaterniond>> &vQ, MatrixXd &V) {
    int m = F.rows();
    //acceleration
    if(!face_unposed) {
        V_EG_unposed.resize(V_EG_pose.rows(), 3);
        fill_egQs();

        //unpose given example using possion stitching
        for (int i = 0; i < num_eg; ++i) {
            vector<MatrixXd> R_per_face = get_avg_R_per_face(face_skinning_weight, egQs[i]);
            MatrixXd B(3 * m, 3);
            for (int i = 0; i < m; ++i) {
                MatrixXd R = R_per_face[i].transpose();

                B.row(i) = R.row(0);
                B.row(m + i) = R.row(1);
                B.row(2 * m + i) = R.row(2);
            }
            //find vertex_unposed position for point j
            MatrixXd V_pose = V_EG_pose.block(i * V.rows(), 0, V.rows(), 3);
            MatrixXd V_unpose(V_pose.rows(), 3);
            possion_stitching(false, V_pose, B, V_unpose);
            V_EG_unposed.block(i * V_unpose.rows(), 0, V_unpose.rows(), 3) = V_unpose;
        }

        //compute deformation gradient
        for (int i = 0; i < num_eg; ++i) {
            MatrixXd V_unpose = V_EG_unposed.block(i * V.rows(), 0, V.rows(), 3);
            MatrixXd N_unpose, N;
            per_face_normals(V_original, F, N);
            per_face_normals(V_unpose, F, N_unpose);
            vector<MatrixXd> T_i;
            Vector3d q1, q2, q3, q1_prime, q2_prime, q3_prime;
            MatrixXd Q(3, 3), Q_prime(3, 3);

            for (int j = 0; j < F.rows(); ++j) {
                MatrixXd T_i_j(3, 3);
                q1 = V_original.row(F(j, 0));
                q2 = V_original.row(F(j, 1));
                q3 = V_original.row(F(j, 2));
                Q.col(0) = q1 - q3;
                Q.col(1) = q2 - q3;
                Q.col(2) = N.row(j);
                q1_prime = V_unpose.row(F(j, 0));
                q2_prime = V_unpose.row(F(j, 1));
                q3_prime = V_unpose.row(F(j, 2));
                Q_prime.col(0) = q1_prime - q3_prime;
                Q_prime.col(1) = q2_prime - q3_prime;
                Q_prime.col(2) = N_unpose.row(j);
                //we need T w.r.t p.T(1x3)!!
                T_i_j = (Q_prime * Q.inverse()).transpose();
                T_i.push_back(T_i_j);
            }
            deformation_gradient.push_back(T_i);
        }

        face_unposed = true;
    }

    //if only visualize unposed shape
    if (visualization_mode == UNPOSED) {
        if (unposed_shape >= num_eg) {
            cout <<"invalid vertex_unposed shape" << endl;
            return;
        }
        V = V_EG_unposed.block(unposed_shape * V.rows(), 0, V.rows(), 3);
        return;
    }

    VectorXd a = get_a();
    vector<MatrixXd> R_per_face = get_avg_R_per_face(face_skinning_weight, vQ);
    MatrixXd B(F.rows() * 3, 3);

    for (int i = 0; i < F.rows(); ++i) {
        MatrixXd R = R_per_face[i];
        // deformation gradient for a face
        if (!decompose_T) {
            Matrix3d T(3, 3);
            T.setZero();
            for (int j = 0; j < num_eg; ++j) {
                T += a[j] * deformation_gradient[j][i];
            }
            //remember now the rotation is for p.T(1x3)
            R = T * R;
        } else {
            //decompose T into rotation and skew matrix
            Matrix3d S(3, 3);
            Matrix3d Q_avg(3, 3);
            S.setZero();
            vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> q_list;
            for (int j = 0; j < num_eg; ++j) {
                //already w.r.t to p.T(1x3)
                Matrix3d Q_j_f, S_j_f;
                //if buffer not full
                if (S_j_f_list.size() < F.rows() * num_eg) {
                    //decompose T into rotation and skew matrix
                    decomposition(deformation_gradient[j][i], Q_j_f, S_j_f);
                    S_j_f_list.push_back(S_j_f);
                    Q_j_f_list.push_back(Q_j_f);
                } else {
                    S_j_f = S_j_f_list[i * num_eg + j];
                    Q_j_f = Q_j_f_list[i * num_eg + j];
                }
                S += a[j] * S_j_f;
                //w.r.t p.T(1x3)
                Quaterniond q(Q_j_f);
                q_list.push_back(q);
            }
            Q_avg = get_avg_rotation(a, q_list);
            R = S * Q_avg * R;
        }
        B.row(i) = R.row(0);
        B.row(m + i) = R.row(1);
        B.row(2 * m + i) = R.row(2);
    }

    //acceleration
    //possion stitching
    bool using_prefactored;
    //if not prefactored then just do prefactoring!
    if (!prefactored) {
        using_prefactored = false;
        prefactored = true;
    } else {
        using_prefactored = true;
    }

    possion_stitching(using_prefactored, V_original, B, V);
}

//decompose T into rotation and skew matrix
void decomposition(const MatrixXd &T, Matrix3d &R, Matrix3d &S) {
    //(RS).T * (RS) = S.TS = V.T D V
    Eigen::Matrix3d M = T.transpose() * T;

    //skew matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(M);
    Eigen::Matrix3d D = solver.eigenvalues().asDiagonal();
    Eigen::Matrix3d V = solver.eigenvectors();
    S = V * D.cwiseSqrt() * V.transpose();

    R = T * S.inverse();
}

//fill the container for each example
void fill_egQs() {
    for (int i = 0; i < num_eg; ++i) {
        //relative quaternion for each example
        std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> dQ;
        //global quaternion and transformation for each example
        std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> egQ;
        vector<Eigen::Vector3d> egT;
        for (int k = 0; k < BE.rows(); ++k) {
            dQ.push_back(q_EG_list[i * BE.rows() + k]);
        }
        igl::forward_kinematics(C_original, BE, P, dQ, egQ, egT);
        egQs.push_back(egQ);
        egTs.push_back(egT);
    }
}

//get a_j
VectorXd get_a() {
    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> current_egQ;
    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> other_egQ;
    vector<Eigen::Vector3d> current_egT;
    vector<Eigen::Vector3d> other_egT;
    //acceleration
    //didn't compute c yet
    if (!RBF_interpolated) {
        //RBF coeff
        c.resize(num_eg, num_eg);
        //RBF interpolation to get
        VectorXd b(num_eg);
        b.setZero();
        for (int i = 0; i < num_eg; ++i) {
            b[i] = 1;
            MatrixXd A(num_eg, num_eg);
            for (int j = 0; j < num_eg; ++j) {
                current_egQ = egQs[j];
                current_egT = egTs[j];
                for (int k = 0; k < num_eg; ++k) {
                    other_egQ = egQs[k];
                    other_egT = egTs[k];
                    A(j, k) = get_RBF_distance(current_egQ, other_egQ, current_egT, other_egT);
                }
            }
            VectorXd ci = A.colPivHouseholderQr().solve(b);
            c.row(i) = ci;
        }
        RBF_interpolated = true;
    }

    //compute weight
    VectorXd current_RBF_distance(num_eg);
    for (int i = 0; i < num_eg; ++i) {
        other_egQ = egQs[i];
        other_egT = egTs[i];
        current_RBF_distance(i) = get_RBF_distance(vQ, other_egQ, vT, other_egT);
    }
    VectorXd a = c * current_RBF_distance;
    //L1 normalize
    a /= a.sum();

    assert(abs(a.sum() - 1) < 1e-6);
    return a;
}

//get distance between P and P_j
double get_RBF_distance(const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> &Q0,
                        const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> &Q1,
                        const vector<Eigen::Vector3d> &T0,
                        const vector<Eigen::Vector3d> &T1) {
    double sum = 0;
    assert(Q0.size() == Q1.size());
    assert(Q0.size() == T0.size());
    MatrixXd M0 = MatrixXd::Identity(4, 4);
    MatrixXd M1 = MatrixXd::Identity(4, 4);
    //Frobenius norm between transformation matrix
    for (int i = 0; i < Q0.size(); ++i) {
        Quaterniond q0 = Q0[i];
        Quaterniond q1 = Q1[i];

        //+-q correspond to the same Rotation matrix
        M0.block<3, 3>(0, 0) = q0.toRotationMatrix();
        M1.block<3, 3>(0, 0) = q1.toRotationMatrix();

        Vector3d t0 = T0[i];
        Vector3d t1 = T1[i];
        M0.block<3, 1>(0, 3) = t0;
        M1.block<3, 1>(0, 3) = t1;
        /*
         * not smooth enough if using L2 distance which will amplified error
         *
         */
//        sum += (M0 - M1).squaredNorm();
        //choose L1 distance
        sum += (M0 - M1).norm();
    }

    double r = sqrt(sum);
    //using r^3 as RBF kernel
    return r * r * r;
}

//fusing transformation of each joint and skinning weight for each vertex
MatrixXd get_fusing_transformation(int j, const MatrixXd &vertex_skinning_weight, const vector<Eigen::Quaterniond,
        Eigen::aligned_allocator<Eigen::Quaterniond>> &vQ, const vector<Eigen::Vector3d> &vT) {
    assert(vQ.size() == BE.rows());
    //fusing transformation
    MatrixXd T(4, 4);
    T.setZero();
    for (int k = 0; k < BE.rows(); ++k) {
        MatrixXd current_T = MatrixXd::Identity(4, 4);

        MatrixXd R = vQ[k].toRotationMatrix();
        current_T.block<3, 3>(0, 0) = R;
        current_T.block<3, 1>(0, 3) = vT[k];
        current_T *= vertex_skinning_weight(k, j);
        T += current_T;
    }
    return T;
}

//quaternion avg for all faces
vector<MatrixXd> get_avg_R_per_face(const MatrixXd &face_skinning_weight, const vector<Eigen::Quaterniond,
Eigen::aligned_allocator<Eigen::Quaterniond>> &vQ) {
    vector<MatrixXd> R_per_face;
    int m = F.rows();
    for (int i = 0; i < m; ++i) {
        //remember to transpose!!! we need R w.r.t p.T(1x3)
        MatrixXd R = get_avg_rotation(face_skinning_weight.col(i), vQ).transpose();
        R_per_face.push_back(R);
    }
    return R_per_face;
}

//quaternion avg
MatrixXd get_avg_rotation(const VectorXd &weight, const vector<Eigen::Quaterniond,
        Eigen::aligned_allocator<Eigen::Quaterniond>> &vQ) {
    assert(weight.size() == vQ.size());
    MatrixXd S(4, 4);
    S.setZero();
    for (int k = 0; k < vQ.size(); ++k) {
        Vector4d coeff = vQ[k].coeffs();
        //[x, y, z, w]
        MatrixXd s = coeff * coeff.transpose();
        S += weight[k] * s;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(S);
    //pick the one correspond to the largest eigen_value
    Vector4d r = solver.eigenvectors().col(3);
    //[x, y, z, w]
    Quaterniond q_avg(r(3), r(0), r(1), r(2));
    return q_avg.toRotationMatrix();
}

//just derive the same formula as assignment 5
//V.T G.TG V-2 V.T G.TB
// H = G.TG E = G.TB
//derivative w.r.t. Vf => HffVf = -HfcVc + Ef
void possion_stitching(bool use_prefactored, const MatrixXd &V_old, const MatrixXd &B, MatrixXd &V_new) {
    /*
     * the function will overwrite the global variable!!!!
     */
    //acceleration
    if (!use_prefactored) {
        SparseMatrix<double> H;
        igl::grad(V_old, F, G);
        constrain_points.resize(V_old.rows());
        free_points.resize(V_old.rows());
        int constrain_cnt = 0;
        int free_cnt = 0;
        int root_bone_id = 0;
        // here try to pick a pone with least moving as root_bone to fix
        // reverse to search for root bone
        // because the first root_bone intend to stay static
        // also skip the root_bone with no given handles !!!!!!!!
        for (int k = P.rows() - 1; k >= 0; --k) {
            if (P(k) == -1) {
                for (int i = 0; i < handle_id.size(); ++i) {
                    if (handle_id[i] == k) {
                        root_bone_id = k;
                        break;
                    }
                }
            }
        }
        for (int i = 0; i < handle_id.size(); ++i) {
            //root handles
            if (handle_id[i] == root_bone_id) {
                constrain_points[constrain_cnt++] = i;
            } else {
                free_points[free_cnt++] = i;
            }
        }
        constrain_points.conservativeResize(constrain_cnt);
        free_points.conservativeResize(free_cnt);

        H = G.transpose() * G;

        slice(H, free_points, free_points, Hff);
        slice(H, free_points, constrain_points, Hfc);

        slice(V_old, constrain_points, 1, V_c);

        solver.compute(Hff);
    }

    MatrixXd E, V_f, E_f, rhs;
    E = G.transpose() * B;
    slice(E, free_points, 1, E_f);

    rhs = -Hfc * V_c + E_f;

    V_f = solver.solve(rhs);
    slice_into(V_f, free_points, 1, V_new);
    slice_into(V_c, constrain_points, 1, V_new);
}
