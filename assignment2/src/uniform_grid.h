#include <vector>
#include <Eigen/Core>
using namespace std;

struct GridCell {
    //store the point index of the gridcell
    vector<int> point_index;    
};

class UniformGrid {
private:
    vector<GridCell> cells;
    Eigen::Vector3d min_corner_point;
    //same cell_size in 3 dimension
    double cell_size;
    //resolution is not the same
    int resolution[3];
    //all the points
    Eigen::MatrixXd P;

    vector<int> getPossiblePointIndex(const Eigen::RowVector3d& point) {
        int x = (point[0] - min_corner_point[0]) / cell_size;
        int y = (point[1] - min_corner_point[1]) / cell_size;
        int z = (point[2] - min_corner_point[2]) / cell_size;
        vector<int> possible_point_index;
        //check the 26 cells around the target cell
        for(int i = x - 1; i < x + 2; ++i) {
            for(int j = y - 1; j < y + 2; ++j) {
                for(int k = z - 1; k < z + 2; ++k) {
                    if((i >= 0 && i < resolution[0] && j >= 0 && j < resolution[1] && k >= 0 && k < resolution[2])) {
                        int current_cell_index = i + j * resolution[0] + k * resolution[0] * resolution[1];
                        vector<int> current_cell_point_index = cells[current_cell_index].point_index;
                        possible_point_index.insert(possible_point_index.end(), current_cell_point_index.begin(), current_cell_point_index.end());
                    }
                }
            }
        }
        return possible_point_index;
    }

public:
    UniformGrid(const Eigen::MatrixXd& P, double cell_size) : cell_size(cell_size), P(P) {
        Eigen::RowVector3d max_corner_point = P.colwise().maxCoeff();
        min_corner_point = P.colwise().minCoeff();
        
        for(unsigned i = 0; i < 3; ++i) {
            resolution[i] = (max_corner_point[i] - min_corner_point[i]) / cell_size + 1;
        }
        cells.resize(resolution[0] * resolution[1] * resolution[2]);

        
        for(unsigned i = 0; i < P.rows(); ++i) {
            int x = (P(i, 0) - min_corner_point[0]) / cell_size;
            int y = (P(i, 1) - min_corner_point[1]) / cell_size;
            int z = (P(i, 2) - min_corner_point[2]) / cell_size;
            int cell_index = x + y * resolution[0] + z * resolution[0] * resolution[1];
            cells[cell_index].point_index.push_back(i);
        }
    }

    bool isClosest(const Eigen::RowVector3d& point, int target_index) {
        vector<int> possible_point_index = getPossiblePointIndex(point);
        //make sure the closest
        double min_dist = (point - P.row(target_index)).squaredNorm();
        for(auto i : possible_point_index) {
            double dist = (point - P.row(i)).squaredNorm();
            if(dist < min_dist || (dist == min_dist && i != target_index)) {
                return false;
            }
        }
        return true;
    }

    vector<int> getNeighbors(const Eigen::RowVector3d& point, double h) {
        vector<int> possible_point_index = getPossiblePointIndex(point);
        vector<int> final_point_index;
        for(auto i : possible_point_index) {
            double dist = (point - P.row(i)).squaredNorm();
            if(dist <= h * h) {
                final_point_index.push_back(i);
            }
        }
        return final_point_index;
    }

};