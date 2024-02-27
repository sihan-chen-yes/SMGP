#include <iostream>
#include <igl/readOFF.h>
#include <imgui.h>
#include "viewer_proxy.h"

/*** insert any libigl headers here ***/
#include <igl/vertex_triangle_adjacency.h>
#include <igl/adjacency_list.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/per_corner_normals.h>
#include <igl/facet_components.h>
#include <igl/jet.h>
#include <igl/edge_topology.h>
#include <igl/barycenter.h>
#include <igl/is_border_vertex.h>


using namespace std;

// Vertex array, #V x3
Eigen::MatrixXd V;
// Face array, #F x3
Eigen::MatrixXi F; 
// Per-face normal array, #F x3
Eigen::MatrixXd FN; 
// Per-vertex normal array, #V x3
Eigen::MatrixXd VN;
// Per-corner normal array, (3#F) x3
Eigen::MatrixXd CN;
// Vectors of indices for adjacency relations
std::vector<std::vector<int> > VF, VFi, VV;
// Integer vector of component IDs per face, #F x1
Eigen::VectorXi cid;
// Per-face color array, #F x3
Eigen::MatrixXd component_colors_per_face;

void subdivide_sqrt3(const Eigen::MatrixXd &V,
					 const Eigen::MatrixXi &F,
					 Eigen::MatrixXd &Vout,
					 Eigen::MatrixXi &Fout){
    //step1 cal barycenter
    Eigen::MatrixXd BC;
	igl::barycenter(V, F, BC);

    //step2 avg vertex and vstack to gen Vout
    Vout.resize(V.rows() + BC.rows(), V.cols());
    igl::adjacency_list(F, VV);
    vector<bool> isBoundary;
    isBoundary = igl::is_border_vertex(F);
    for (int i = 0; i < V.rows(); ++i) {
        Eigen::RowVectorXd r = V.row(i);
        if (!isBoundary[i]) {
            int n = VV[i].size();
            double a_n = (4 - 2 * cos(2 * M_PI / n)) / 9;
            r *= (1 - a_n);
            for (int j : VV[i]) {
                r += (a_n / n) * V.row(j);
            }
        }
        Vout.row(i) = r;
    }
    for (int i = 0; i < BC.rows(); ++i) {
        Vout.row(V.rows() + i) = BC.row(i);
    }

    //step3 gen Fout and flip edge
    Fout.resize(3 * F.rows(), F.cols());
    for (int i = 0; i < F.rows(); ++i) {
        int v0 = F.row(i)[0];
        int v1 = F.row(i)[1];
        int v2 = F.row(i)[2];
        int vc = V.rows() + i;

        Fout.row(3 * i + 0) << v0, vc, v2;
        Fout.row(3 * i + 1) << v0, v1, vc;
        Fout.row(3 * i + 2) << v1, v2, vc;
    }

    //flip edge
    Eigen::MatrixXi EV, FE, EF;
    igl::edge_topology(Vout, Fout, EV, FE, EF);
    for (int i = 0; i < EV.rows(); ++i) {
        int v0 = EV.row(i)[0];
        int v1 = EV.row(i)[1];
        int f0 = EF.row(i)[0];
        int f1 = EF.row(i)[1];
        //only flip 1.not being newly added 2.not being boundary edge
        if (v0 < V.rows() && v1 < V.rows() && EF.row(i)[0] != -1 && EF.row(i)[1] != -1) {
            //find two side vertex
            int v2, v3;
            for (int j : Fout.row(f0)) {
                if (j != v0 && j != v1) {
                    v2 = j;
                }
            }
            for (int j : Fout.row(f1)) {
                if (j != v0 && j != v1) {
                    v3 = j;
                }
            }
            Fout.row(f0) << v2, v3, v1;
            Fout.row(f1) << v2, v0, v3;
        }
    }
}

void print(std::vector<std::vector<int> > container) {
    for (size_t i = 0; i < container.size(); ++i) {
        cout << i << ":";
        for (size_t j = 0; j < container[i].size(); ++j) {
            cout << container[i][j];
            if (j < container[i].size() - 1) {
                cout << ", ";
            }
        }
        cout << endl;
    }
}

void printMatrix(Eigen::MatrixXi matrix) {
    for (int i = 0; i < matrix.rows(); ++i) {
        cout << i << ": ";
        for (int j = 0; j < matrix.cols(); ++j) {
            cout << matrix(i, j);
            if (j < matrix.cols() - 1) {
                cout << ", ";
            }
        }
        cout << endl;
    }
}

bool callback_key_down(ViewerProxy& viewer, unsigned char key, int modifiers) {
    if (key == '1') {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        // Add your code for computing vertex to face relations here;
        // store in VF,VFi.
        igl::vertex_triangle_adjacency(V, F, VF, VFi);
        cout << "VF:" << endl;
        print(VF);
        cout << "VFi:" << endl;
        print(VFi);
        cout << "F:" << endl;
        printMatrix(F);
    }

    if (key == '2') {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        // Add your code for computing vertex to vertex relations here:
        // store in VV.
        igl::adjacency_list(F, VV);
        cout << "VV:" << endl;
        print(VV);
    }

    if (key == '3') {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        FN.setZero(F.rows(),3);
        // Add your code for computing per-face normals here: store in FN.
        igl::per_face_normals(V, F, FN);
        // Set the viewer normals.
        viewer.data().set_normals(FN);
    }

    if (key == '4') {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        // Add your code for computing per-vertex normals here: store in VN.
        igl::per_vertex_normals(V, F, VN);
        // Set the viewer normals.
        viewer.data().set_normals(VN);
    }

    if (key == '5') {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        // Add your code for computing per-corner normals here: store in CN.
        double threshold = 10.0;
        igl::per_corner_normals(V, F, threshold, CN);
        //Set the viewer normals
        viewer.data().set_normals(CN);
    }

    if (key == '6') {
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        component_colors_per_face.setZero(F.rows(),3);
        // Add your code for computing per-face connected components here:
        igl::facet_components(F, cid);
        // Compute colors for the faces based on components, storing them in
        // component_colors_per_face.
        igl::jet(cid, true, component_colors_per_face);
        // Set the viewer colors
        viewer.data().set_colors(component_colors_per_face);
    }

    if (key == '7') {
		Eigen::MatrixXd Vout;
		Eigen::MatrixXi Fout;
        // Fill the subdivide_sqrt3() function with your code for sqrt(3) subdivision.
		subdivide_sqrt3(V,F,Vout,Fout);
        // Set up the viewer to display the new mesh
        V = Vout; F = Fout;
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
    }

    return true;
}

bool load_mesh(ViewerProxy& viewer,string filename, Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
  igl::readOFF(filename,V,F);
  viewer.data().clear();
  viewer.data().set_mesh(V,F);
  viewer.data().compute_normals();
  viewer.core().align_camera_center(V, F);
  return true;
}

int main(int argc, char *argv[]) {
    // Show the mesh
    ViewerProxy& viewer = ViewerProxy::get_instance();
    viewer.callback_key_down = callback_key_down;
    
    std::string filename;
    if (argc == 2) {
        filename = std::string(argv[1]); // Mesh provided as command line argument
    }
    else {
        filename = std::string("../data/bunny.off"); // Default mesh
    }
	
    load_mesh(viewer,filename,V,F);

    callback_key_down(viewer, '1', 0);
    
    viewer.launch();
}
