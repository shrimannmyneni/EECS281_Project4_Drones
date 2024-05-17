// Project identifier: 1761414855B69983BD8035097EFBD312EB0527F0

#include <iostream>
#include <getopt.h>
#include <vector>
#include <cmath>
#include <limits>
#include <iomanip>
#include <cassert>

enum class CampusType : uint8_t {
    MAIN,
    MEDICAL,
    BORDER
};

struct Location{
    int x;
    int y;
    CampusType campus_type;
    size_t vertex_index;
    size_t locations_iv_index;
};



class Drone {
    private:
    std::string mode;
    size_t number_of_delivery_locations;

    //PART A: MST
    double mst_final_path_length = 0;
    
    //PRIMS
    struct Prims{
        const Location& v; //v
        bool visited; //k
        double distance_sq; //d
        int parent_index; //p

        Prims(const Location& _v)
            : v(_v), visited(false), distance_sq(std::numeric_limits<double>::infinity()),
            parent_index(-1){}
            
    };
    std::vector<Prims> prims_table;
    //PRIMS

    std::vector<Location> locations_iv;

    public:
    void get_options(int argc, char* argv[]);
    void read_input();

    //helper functions
    double distance_helper(const Location& a, const Location& b);
    double distance_squared_helper(const Location& a, const Location& b);
    void which_algorithm();
    void output();

    //PART A
    void prims_algorithm();

    //PART B
    std::vector<Location> fast_tsp_path_vector;
    void fast_tsp();
    // double fast_distance_helper(const Location* a, const Location* b);
    double fast_tsp_path_length = 0;
    double current_fast_tsp_path_length = 0;

    //PART C 
    // struct vertexC{
    //     int x;
    //     int y;
    //     size_t idx;

    //     vertexC(int _x, int _y, size_t _idx): x(_x), y(_y), idx(_idx){}
    // };
    std::vector<Location> opt_tsp_path_vector;
    std::vector<Location> best_opt_tsp_path_vector;
    void opt_tsp();
    double opt_tsp_path_length = 0.0;
    double curr_best_opt_tsp_path_length = std::numeric_limits<double>::infinity();
    bool is_promising(size_t gen_perm_len);
    double calculate_mst_distance_for_remaining_locations();
    
   double total_path_distance(const std::vector<Location>& path, size_t path_len, bool is_complete) {
        double distance = 0.0;
        if (path_len == 0 || path.empty()) return 0.0;  // No path to calculate.

        for (size_t i = 1; i < path_len; ++i) {
            distance += sqrt(distance_helper(path[i - 1], path[i]));
        }

        if (is_complete && path_len > 0 && path_len <= path.size()) {
            distance += sqrt(distance_helper(path[path_len - 1], path[0]));
        }

        return distance;
}

    void generate_permutations(size_t path_len);
    

};


void Drone::get_options(int argc, char* argv[]){
        int optionIdx = 0, option = 0;
        opterr = false;

        struct option longOpts [] = {
            {"help", no_argument, nullptr, 'h'},
            {"mode", required_argument, nullptr, 'm'},
            {nullptr, no_argument, nullptr, '\0'}
        };
        
    while ((option = getopt_long(argc, argv, "hm:", longOpts, &optionIdx)) != -1){
            switch(option){
                case 'h':
                    std::cerr << "help\n";
                    exit(0);

                case 'm':
                    mode = optarg;
                    

                    if (mode == "MST" || mode == "OPTTSP" || mode == "FASTTSP"){
                        std::cerr << "Mode: " << mode << '\n';
                        break;
                    } else {
                        std::cerr << "Unknown / Invalid Mode\n";
                        exit(1);
                    }
                    
                default:
                    std::cerr << "Unknown / Invalid Option\n";
                    exit(1);
            }
        }
        
    }//end of get_options

void Drone::read_input() {
    std::cin >> number_of_delivery_locations;
    locations_iv.resize(number_of_delivery_locations);

    for (size_t i = 0; i < number_of_delivery_locations; ++i) {
        std::cin >> locations_iv[i].x >> locations_iv[i].y;
        locations_iv[i].vertex_index = i;
    }

    if (mode == "MST") {
        bool mainExists = false;
        bool medicalExists = false;
        bool borderExists = false;

        for (auto& location : locations_iv) {
            if (location.x > 0 || location.y > 0) {
                location.campus_type = CampusType::MAIN;
                mainExists = true;
            } else if (location.x < 0 && location.y < 0) {
                location.campus_type = CampusType::MEDICAL;
                medicalExists = true;
            } else {
                location.campus_type = CampusType::BORDER;
                borderExists = true;
            }
        }

        if (medicalExists && mainExists && !borderExists) {
            std::cerr << "Cannot construct MST\n";
            exit(1);
        }
    }

    which_algorithm();
}

    double Drone::distance_helper(const Location& a, const Location& b){
        double xDiff = a.x - b.x;
        double yDiff = a.y - b.y;
        return sqrt((xDiff*xDiff)+(yDiff*yDiff));
        //look at how to optimize checking for sqrt() / distance
    }

    double Drone::distance_squared_helper(const Location& a, const Location& b){
        if (mode == "MST") {
            if ((a.campus_type == CampusType::MAIN && b.campus_type == CampusType::MEDICAL) 
            || (a.campus_type == CampusType::MEDICAL && b.campus_type == CampusType::MAIN)) {
                return std::numeric_limits<double>::infinity();
            }
        }

        double xDiff = a.x - b.x;
        double yDiff = a.y - b.y;
        return (xDiff*xDiff)+(yDiff*yDiff);
    }


    void Drone::which_algorithm(){
        if (mode == "MST"){
            prims_algorithm();
        } else if (mode == "FASTTSP"){
            fast_tsp();
        } else if (mode == "OPTTSP"){
            opt_tsp();
        }
    }

    

    //PART A algorithms

    void Drone::prims_algorithm(){
        //initialize prims table
        prims_table.reserve(number_of_delivery_locations);
        for (const auto& location : locations_iv){
            prims_table.emplace_back(location);
        }

        //initialize starting point: set distance to 0, parent to -1
        prims_table[0].distance_sq = 0;
        prims_table[0].parent_index = -1;

        for (size_t i = 0; i < number_of_delivery_locations; ++i){ // beginning of actual prims
            double minimum_distance_found_so_far = std::numeric_limits<double>::infinity();
            size_t pathfinder_idx = 0;

            for (size_t k = 0; k < number_of_delivery_locations; ++k){
                if (!prims_table[k].visited){
                    if (prims_table[k].distance_sq < minimum_distance_found_so_far){
                        minimum_distance_found_so_far = prims_table[k].distance_sq;
                        pathfinder_idx = k;
                    }
                }
            }

                prims_table[pathfinder_idx].visited = true;
                mst_final_path_length += sqrt(prims_table[pathfinder_idx].distance_sq);
                
                for (size_t v = 0; v < number_of_delivery_locations; ++v){
                    if (!prims_table[v].visited){
                    double length_to_newest_point_on_mst = distance_squared_helper(locations_iv[pathfinder_idx], locations_iv[v]);
                        if (length_to_newest_point_on_mst < prims_table[v].distance_sq){
                            prims_table[v].distance_sq = length_to_newest_point_on_mst;
                            prims_table[v].parent_index = static_cast<int>(pathfinder_idx);
                        }
                    }
                }

        } // outer loop of prims    

    }


    void Drone::fast_tsp() {
        fast_tsp_path_vector.reserve(number_of_delivery_locations + 1);
        assert(number_of_delivery_locations >= 3);
        
        //initialize
        for (size_t i = 0; i < 3; ++i){
            fast_tsp_path_vector.push_back(locations_iv[i]);
        }
        fast_tsp_path_vector.push_back(locations_iv[0]);

        current_fast_tsp_path_length = 0.0;
        for (size_t i = 0; i < 3; ++i) {
            current_fast_tsp_path_length += distance_helper(fast_tsp_path_vector[i], fast_tsp_path_vector[i + 1]);
        }

        double distanceSaved = 0.0;
        double possibleDistance = 0.0;
        size_t insertion_idx;
        for (size_t i = 3; i < number_of_delivery_locations; ++i) {

            double initial_broken_dist = distance_helper(fast_tsp_path_vector[0], fast_tsp_path_vector[1]);
            double initial_add_dist = distance_helper(fast_tsp_path_vector[0], locations_iv[i]) +
                                    distance_helper(locations_iv[i], fast_tsp_path_vector[1]);

            
            distanceSaved = current_fast_tsp_path_length;
            current_fast_tsp_path_length = current_fast_tsp_path_length + initial_add_dist - initial_broken_dist;
            insertion_idx = 1;

            for (size_t j = 2; j < i + 1; ++j) {
                double broken_dist = distance_helper(fast_tsp_path_vector[j - 1], fast_tsp_path_vector[j]);
                double added_dist = distance_helper(locations_iv[i], fast_tsp_path_vector[j - 1]) +
                                distance_helper(locations_iv[i], fast_tsp_path_vector[j]);
                possibleDistance = distanceSaved - broken_dist + added_dist;

                if (current_fast_tsp_path_length > possibleDistance) {
                    current_fast_tsp_path_length = possibleDistance;
                    insertion_idx = j;
                }
            }
            fast_tsp_path_vector.insert(fast_tsp_path_vector.begin() + static_cast<std::ptrdiff_t>(insertion_idx), locations_iv[i]);
            }
    }

    //recursive (genPerms)
    void Drone::opt_tsp(){
        fast_tsp();
        curr_best_opt_tsp_path_length = current_fast_tsp_path_length;
        std::reverse(fast_tsp_path_vector.begin(), fast_tsp_path_vector.end());
        fast_tsp_path_vector.pop_back(); // right
        opt_tsp_path_vector = fast_tsp_path_vector;
        generate_permutations(1);
    }

    void Drone::generate_permutations(size_t path_len) {
        if (path_len == opt_tsp_path_vector.size()) {
            double d = distance_helper(opt_tsp_path_vector[path_len - 1], opt_tsp_path_vector[0]);
            opt_tsp_path_length += d;
            
            if (opt_tsp_path_length < curr_best_opt_tsp_path_length) {
                curr_best_opt_tsp_path_length = opt_tsp_path_length;
                best_opt_tsp_path_vector = opt_tsp_path_vector;
            }

            opt_tsp_path_length -= d;
            return;
        }

        if (!is_promising(path_len)) {
            return;
        }

        for (size_t i = path_len; i < opt_tsp_path_vector.size(); ++i) {
            std::swap(opt_tsp_path_vector[path_len], opt_tsp_path_vector[i]);
            double dist = distance_helper(opt_tsp_path_vector[path_len], opt_tsp_path_vector[path_len - 1]);
            opt_tsp_path_length += dist;
            generate_permutations(path_len + 1);
            opt_tsp_path_length -= dist;
            std::swap(opt_tsp_path_vector[path_len], opt_tsp_path_vector[i]);
        }
}


  bool Drone::is_promising(size_t gen_perm_len) {
    if (opt_tsp_path_vector.size() - gen_perm_len < 5) {
        return true;
    }

    if (opt_tsp_path_length >= curr_best_opt_tsp_path_length) {
        return false;
    }

    prims_table.clear();
    prims_table.reserve(opt_tsp_path_vector.size() - gen_perm_len);
    for (size_t i = gen_perm_len; i < opt_tsp_path_vector.size(); ++i) {
        prims_table.emplace_back(Prims(opt_tsp_path_vector[i]));
    }

    double mst_estimated_distance = calculate_mst_distance_for_remaining_locations();

    double min_distance_to_unvisited = std::numeric_limits<double>::infinity();
    for (size_t i = gen_perm_len; i < number_of_delivery_locations; ++i) {
        double distance = distance_helper(opt_tsp_path_vector[0], opt_tsp_path_vector[i]);
        if (min_distance_to_unvisited > distance) {
            min_distance_to_unvisited = distance;
        }
    }

    double min_distance_to_unvisited2 = std::numeric_limits<double>::infinity();
    for (size_t i = gen_perm_len; i < number_of_delivery_locations; ++i) {
        double distance = distance_helper(opt_tsp_path_vector[gen_perm_len - 1], opt_tsp_path_vector[i]);
        if (min_distance_to_unvisited2 > distance) {
            min_distance_to_unvisited2 = distance;
        }
    }

    double expected_lower_bound = opt_tsp_path_length + mst_estimated_distance + min_distance_to_unvisited + min_distance_to_unvisited2;

    return expected_lower_bound < curr_best_opt_tsp_path_length;
}


    double Drone::calculate_mst_distance_for_remaining_locations() {
        double total_mst_distance = 0.0;

        prims_table[0].distance_sq = 0.0;
        size_t pathfinder_idx = 0;
        //std::cout << "Prims table size: " << prims_table.size() << '\n';

        for (size_t i = 0; i < prims_table.size(); ++i) {
            double minimum_distance_found = std::numeric_limits<double>::infinity();
            
            for (size_t k = 0; k < prims_table.size(); ++k) {
                if (!prims_table[k].visited && prims_table[k].distance_sq < minimum_distance_found) {
                    minimum_distance_found = prims_table[k].distance_sq;
                    pathfinder_idx = k;
                }
            }

            prims_table[pathfinder_idx].visited = true;
            total_mst_distance += sqrt(prims_table[pathfinder_idx].distance_sq);

            for (size_t v = 0; v < prims_table.size(); ++v) {
                if (!prims_table[v].visited) {
                    double length_to_new_point = distance_squared_helper(opt_tsp_path_vector[pathfinder_idx], opt_tsp_path_vector[v]);
                    if (length_to_new_point < prims_table[v].distance_sq) {
                        prims_table[v].distance_sq = length_to_new_point;
                    }
                }
            }

        }
        return total_mst_distance;
    }
    

    void Drone::output(){
        if (mode == "MST"){
            std::cout << std::fixed << std::setprecision(2) << mst_final_path_length << '\n';
            
            for (size_t i = 1; i < prims_table.size(); ++i){
                int parent_idx = prims_table[i].parent_index;
                if (parent_idx < static_cast<int>(i)){
                    std::cout << parent_idx << " " << i << '\n';
                } else {
                    std::cout << i << " " << parent_idx << '\n';
                }
            }
        }

        else if (mode == "FASTTSP"){
            std::cout << std::fixed << std::setprecision(2) << current_fast_tsp_path_length << '\n';
            
            for (size_t i = 0; i < fast_tsp_path_vector.size() - 1; ++i){
                std::cout << fast_tsp_path_vector[i].locations_iv_index << " ";
            }
        }

        else if (mode == "OPTTSP"){
            std::cout << std::fixed << std::setprecision(2) << curr_best_opt_tsp_path_length << '\n';

            for (auto& loc : best_opt_tsp_path_vector){
                std::cout << loc.vertex_index << " ";
            }
        }
    }


int main(int argc, char* argv[]){
    std::ios_base::sync_with_stdio(false);
    Drone d;
    d.get_options(argc, argv);
    d.read_input();
    d.output();
    return 0;
}
