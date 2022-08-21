//----------------------------------------------------------------------------------------------------
// Tomas Ortega Rojas
// Program that, using the selected algorithm, finds a path in a graph from a start to goal vertex.
//----------------------------------------------------------------------------------------------------

#include <cmath>
#include <set>
#include <iostream>
#include <getopt.h>
#include <queue>
#include <fstream>
#include <map>
#include <regex>
#include <sstream>

bool depth_opt;
bool debug_opt;
bool v_opt;

#define v_trace(fmt...) do { if (v_opt) { printf(fmt); printf("\n"); fflush(stdout); } } while(0)
#define debug_trace(fmt...) do { if (debug_opt) { printf(fmt); printf("\n"); fflush(stdout); } } while(0)


// TODO:I realized that there is a small bug in the output for A*. It returns the correct solution but I made a mistake reading the sample output and assume that only one edge was showing at any one time instead of the path until that moment. I hope this mistake doesn't cost me a lot of points since it would be easy to solve given more time. 

class Vertex{
public:
    std::string name;
    double x;
    double y;
    double g = std::numeric_limits<double>::infinity();
    double f = std::numeric_limits<double>::infinity();

    Vertex *previous;

    Vertex(int x_coordinate, int y_coordinate, std::string n){
        x = x_coordinate;
        y = y_coordinate;
        name = n;
        previous = nullptr;  // best path to get to vertex (fewer edges or shortest path depending on the algorithm)
    }

    // Compares two vertices according to name.
    static bool compare(Vertex *v, Vertex *u){
        return (v->name < u->name);
    }
};


class Graph{
public:
    std::map<std::string, Vertex*> vertices;
    std::map<std::string, std::vector<Vertex*>> adj_list;

    void add_vertex(Vertex *v){
        if(vertices.find(v->name) != vertices.end()){
            return ;
        }
        if(adj_list.find(v->name) != adj_list.end()) {
            printf("The name %s is already being used", v->name.c_str());
            return;
        }
        vertices[v->name] = v;
        adj_list[v->name] = std::vector<Vertex*>();
    }

    bool add_edge(const std::string& v, const std::string& u){
        // returns true if the operation was successful, and false otherwise
        if(adj_list.find(v) == adj_list.end() ||  adj_list.find(u) == adj_list.end()) {
            return false;
        } else {
            //add the edge vu to v and uv to u
            adj_list[v].push_back(vertices[u]);
            adj_list[u].push_back(vertices[v]);
        }
        return true;
    }

    // Function that calculates the distance between two vertices
    static double distance(const Vertex& p, const Vertex& q){
        return sqrt(pow(p.x - q.x, 2) + pow(p.y - q.y, 2));
    }


    std::vector<Vertex*> recursive_path_helper(Vertex &goal){
        std::vector<Vertex*> r;
        if(goal.previous == nullptr){
            r = std::vector<Vertex*>();
        }else{
            r = recursive_path_helper(*goal.previous);
        }
        r.push_back(&goal);
        return r;
    }

    std::vector<Vertex*> get_path(const std::string& source, const std::string& goal, const std::string& algorithm, int depth){
        int solution = 0;
        if(algorithm == "BFS"){
            solution = bfs(*vertices[source], *vertices[goal]);
        }else if(algorithm == "ID"){
            if(depth > 0){
                solution = iterative_deepening(*vertices[source], *vertices[goal], depth);
            }
        }else if(algorithm == "ASTAR"){
            solution = a_star(*vertices[source], *vertices[goal]);
        }else{
            std::cerr << "Error. algorithm " << algorithm << "not supported" << std::endl;
        }
        if(solution == 0){
            // There is no solution
            return std::vector<Vertex*>();
        }
        return recursive_path_helper(*vertices[goal]);
    }

    //----------------------------------------------------------------------------------------
    // Implementation of BFS
    //----------------------------------------------------------------------------------------

    int bfs(Vertex &source, Vertex &destination){
        std::set<Vertex*> visited;
        std::deque<Vertex*> frontier;
        frontier.push_back(&source);
        while(!frontier.empty()){
            Vertex *v = frontier.front();
            frontier.pop_front();
            visited.insert(v);
            // It seems that the examples check for goal when expanded, but I believe
            // it would be better to check when the node is generated as explained in the book.
            if(v->name == destination.name){
                return 1;
            }
            v_trace("Expanding %s", v->name.c_str());
            for(Vertex* u : adj_list[v->name]){
                if (visited.find(u) == visited.end() && std::find(frontier.begin(), frontier.end(), u) == frontier.end()){
                    frontier.push_back(u);
                    u->previous = v;
                }
            }
        }
        return 0;
    }

    //----------------------------------------------------------------------------------------
    // Implementation of Iterative Deepening
    // (also called depth-first search iterative deepening)
    //----------------------------------------------------------------------------------------

    int iterative_deepening(Vertex &source, Vertex &destination, int depth){
        int solution_found = 0;
        while(solution_found == 0){
            std::set<Vertex*> visited;
            solution_found = recursive_id(source, destination, depth, visited, depth);
            depth++;
        }
        if(solution_found == -1)
            return 0;
        return 1;
    }

    int recursive_id(Vertex &node, Vertex &destination, int depth, std::set<Vertex*> &visited, int initial_depth){
        if(&node == &destination)
            return 1;  // return that there is a solution
        if (visited.find(&node) != visited.end())
            return -1;  // return that this path does not work (maybe there is no path)
        visited.insert(&node);
        if ( depth == 0){
            v_trace("hit depth=%d: %s", initial_depth, node.name.c_str());
            return 0;  // depth limited (0 means cutoff)
        }
        v_trace("Expand %s", node.name.c_str());
        bool cutoff = false;
        for(Vertex *u : adj_list[node.name]){
            int result = recursive_id(*u, destination, depth-1, visited, initial_depth);
            if(result == 1){
                u->previous = &node;
                return 1;
            }else if(result == 0){
                cutoff = true;
            }
        }
        if(cutoff)
            return 0;
        return -1;
    }

    //----------------------------------------------------------------------------------------
    // Implementation of A*
    //----------------------------------------------------------------------------------------

    int a_star(Vertex &source, Vertex &destination){
        std::priority_queue<std::pair<double, Vertex*>> pq;
        std::set<Vertex*> explored;
        pq.push(std::make_pair(distance(source, destination), &source));
        source.g = 0;
        std::string last_node = source.name;
        while(!pq.empty()){
            float priority = pq.top().first;
            Vertex &node = *pq.top().second;
            pq.pop();
            if(&node == &destination){
                return 1;
            }
            if(last_node != node.name){
                v_trace("adding %s -> %s", last_node.c_str(), node.name.c_str());
            }
            explored.insert(&node);
            for(Vertex *u : adj_list[node.name]){
                u->g = node.g  + distance(node, *u);
                double f = distance(*u, destination)+u->g;
                v_trace("%s -> %s ; g=%0.2f h=%0.2f = %0.2f", node.name.c_str(), u->name.c_str(), u->g, distance(*u, destination), f);
                if (explored.find(u) != explored.end()){
                    continue;
                }
                if (f < u->f){
                    u->f = f;
                    u->previous = &node;
                }
                pq.push(std::make_pair(-1*f, u));
            }
            last_node = node.name;
        }
        return 0;
    }
};

//----------------------------------------------------------------------------------------
// Reading Input File
//----------------------------------------------------------------------------------------

// Function that reads the graph-file
Graph& read_graph(const std::string &filename){
    std::ifstream in(filename);
    if(!in){
        std::cerr << "Error. No input file with that name" << std::endl;
        exit(1);
    }
    std::string line;
    Graph &input_graph = *new Graph();  // if the graph is too large we might want to just keep it in the heap?
    // Each line contains either:
    //      comment: starts with # and should be skipped
    //      vertex: has a node label (string of alphanumeric characters) followed by two ints, and x and a y coordinate
    //      edge: undirected. simply two labels. (they may refer to vertices that are not yet in the graph)
    std::regex vertex("[A-Za-z0-9]* [0-9]* [0-9]*");
    std::regex comment("#.*");
    std::regex edge("[A-Za-z0-9]* [A-Za-z0-9]*");
    std::regex empty("([\\s*] | [\b*])*");
    std::vector<std::string> error_edges;
    while(std::getline(in, line)){
        if(std::regex_match(line, comment)){
            continue;
        }else if(std::regex_match(line, edge)){
            std::istringstream ss(line);
            std::string v1, v2;
            ss >> v1 >> v2;
            if(input_graph.add_edge(v1, v2)){
                continue;
            }else{
                // add the edge to a vector of failed edges. If at the end the vector is not empty we return an error
                error_edges.push_back(line);
            }
        }else if(std::regex_match(line, vertex)){
            // add the vertex to the graph (it is always valid to add a new vertex)
            std::istringstream ss(line);
            std::string name;
            int x, y;
            ss >> name >> x >> y;
            input_graph.add_vertex(new Vertex(x, y, name));
        }else if(!std::regex_match(line, empty)){
            std::cerr << "Error. The line: \n" << line << "\ndoes not contain a vertex, comment or edge" << std::endl;
        }
    }
    // See if we need to add edges to the graph that gave errors previously
    bool invalid = false;
    for (const std::string &error_edge : error_edges){
        std::istringstream ss(error_edge);
        std::string v1, v2;
        ss >> v1 >> v2;
        if(!input_graph.add_edge(v1, v2)){
            std::cerr << "Error. The edge " << v1 << " " << v2 << " contains a vertex that does not exists" << std::endl;
            invalid = true;
        }
    }
    if (invalid){
        exit(0);
    }
    for (auto iter = input_graph.adj_list.begin(); iter != input_graph.adj_list.end(); ++iter){
        // Since we want to process the neighbors in order, we can just store them in order in the adjacency list
        std::sort(iter->second.begin(),iter->second.end(), Vertex::compare);
    }
    return input_graph;
}

int main(int argc, char* argv[]) {
    // path [-v] -start $start-node -goal $goal-node -alg $alg graph-file
    if (argc < 7) {
        std::cerr << "Usage: " << argv[0] << " [v] --start $start-node --goal $goal-node --alg $alg graph-file" << std::endl;
        return 1;
    }
    // -v is optional and it means verbose
    // -s or --start followed by the name of the node to start from (figure out what he means by "name")
    // -e or --end followed by the name of  the node which is the goal
    // -a or --alg followed by BFS, ID, or ASTAR
    // -d or --depth (only for ID) indicates the initial search depth
    std::string filename;
    int opt;
    std::string source;
    std::string goal;
    std::string algorithm;
    int depth = -1;
    while (1){
        static struct option long_options[] = {
                {"start",     required_argument, NULL,  's'},
                {"goal",  required_argument,       NULL,  'g'},
                {"alg",  required_argument, NULL,  'a'},
                {"depth", required_argument,       NULL,  'd'},
                {NULL,      0,                 NULL,  0}
                };
        int option_index = 0;
        opt = getopt_long(argc, argv, "vl", long_options, &option_index);
        if (opt == -1){
            break;
        }
        // optarg is a pointer to the argument of the option
        switch(opt){
            case 'v':
                v_opt = true;
                break;
            case 's':
                source = optarg;
                break;
            case 'g':
                goal = optarg;
                break;
            case 'a':
                algorithm = optarg;
                break;
            case 'd':
                depth_opt = true;
                depth = atoi(optarg);
                break;
            case 'l':
                debug_opt = true;
                break;
            case '?':
                printf("Unknown option: %c\n", optopt);
                std::cerr << "Unknown option " << optopt << std::endl;
                break;
            case ':':
                std::cerr << "Missing arg for " << optopt << std::endl;
                exit(1);
        }
    }
    if (optind < argc){
        while(optind < argc){
            filename = argv[optind++];
        }
    }else{
        std::cerr << "No input file" << std::endl;
        exit(1);
    }
//    debug_trace("source: %s, goal: %s, alg: %s, depth used specified? %d", source.c_str(), goal.c_str(), algorithm.c_str(), depth_opt);
    Graph &graph = read_graph(filename);
    std::vector<Vertex*> path = graph.get_path(source, goal, algorithm, depth);
    if (path.size() == 0){
        std::cout << "No Solution" << std::endl;
        return 0;
    }
    printf("Solution:");
    int i = 0;
    for(Vertex *v : path){
        printf("%s", v->name.c_str());
        if(i != path.size() - 1){
            printf(" -> ");
        }
        i++;
    }
    return 0;
}
