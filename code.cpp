#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Class representing a point in the plane
class Point {
public:
    float x, y;

    Point(float x, float y) : x(x), y(y) {}

    // Calculate the Euclidean distance between two points
    float distance(const Point& other) const{
        float dx = x - other.x;
        float dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }
};

// Class representing the wireless network
class WirelessNetwork {
private:
    std::vector<Point> nodes;
    std::vector<std::vector<int>> adjacencyList;

public:
    // Generate a wireless network with randomly distributed nodes
    void generate(float size, int n) {
        nodes.clear();
        adjacencyList.clear();

        // Generate n random points within the given size
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dist(0, size);

        for (int i = 0; i < n; i++) {
            float x = dist(gen);
            float y = dist(gen);
            nodes.emplace_back(x, y);
        }

        // Build the adjacency list based on node distances
        for (int i = 0; i < n; i++) {
            std::vector<int> neighbors;
            for (int j = 0; j < n; j++) {
                if (i != j && nodes[i].distance(nodes[j]) <= 1.0) {
                    neighbors.push_back(j);
                }
            }
            adjacencyList.push_back(neighbors);
        }
    }

    // Perform topology control using the XTC algorithm
    void topologyControl() {
        // Create a new adjacency list for the sparser network
        std::vector<std::vector<int>> sparserAdjacencyList(nodes.size());

        // Perform XTC algorithm to modify the adjacency list
        for (int i = 0; i < nodes.size(); i++) {
            for (int j = 0; j < nodes.size(); j++) {
                if (i != j && isConnected(i, j)) {
                    sparserAdjacencyList[i].push_back(j);
                }
            }
        }

        // Assign the modified adjacency list to the current instance
        adjacencyList = sparserAdjacencyList;
    }

    bool isConnected(int node1, int node2) {
        // Check if node1 and node2 are neighbors in the adjacency list

        // Retrieve the neighbors of node1 from the adjacency list
        std::vector<int>& neighbors = adjacencyList[node1];

        // Iterate through the neighbors and check if node2 is present
        for (int neighbor : neighbors) {
            if (neighbor == node2) {
                // node2 is a neighbor of node1, so they are connected
                return true;
            }
        }

        // If node2 is not found in the neighbors of node1, they are not connected
        return false;
    }



    // Perform compass routing from a source vertex to a destination vertex
    std::vector<int> compassRouting(int source, int destination) const {
        std::vector<int> path;
        path.push_back(source);

        int current = source;
        while (current != destination) {
            float minAngle = std::numeric_limits<float>::max();
            int next = -1;

            // Find the neighbor with the smallest angle
            for (int neighbor : adjacencyList[current]) {
                float angle = std::atan2(nodes[destination].y - nodes[current].y,
                    nodes[destination].x - nodes[current].x) -
                    std::atan2(nodes[neighbor].y - nodes[current].y,
                        nodes[neighbor].x - nodes[current].x);

                if (angle < 0) {
                    angle += 2 * M_PI;
                }

                if (angle < minAngle) {
                    minAngle = angle;
                    next = neighbor;
                }
            }

            if (next == -1) {
                // No valid path found
                return {};
            }

            path.push_back(next);
            current = next;
        }

        return path;
    }

    



    // Get the average degree of the wireless network
    float getAverageDegree() const {
        if (nodes.empty()) {
            return 0.0f;
        }

        int totalDegree = 0;
        for (const auto& neighbors : adjacencyList) {
            totalDegree += neighbors.size();
        }

        return static_cast<float>(totalDegree) / nodes.size();
    }

    // Get the maximum degree of the wireless network
    int getMaximumDegree() const {
           int maxDegree = 0;
        for (const auto& neighbors : adjacencyList) {
            maxDegree = std::max(maxDegree, static_cast<int>(neighbors.size()));
        }
        return maxDegree;
    }
};

// Class for generating random wireless networks
class Loader {
public:
    static WirelessNetwork generateNetwork(float size, int n) {
        WirelessNetwork network;
        network.generate(size, n);
        return network;
    }
};

// Class for running experiments
class Experiments {
public:
    static void run() {
        int networkSize = 2;  // Number of nodes in the network
        float networkSizeLimit = 5.0;  // Size limit of the network
        int source = 0;  // Source vertex for compass routing
        int destination = 1;  // Destination vertex for compass routing

        WirelessNetwork network = Loader::generateNetwork(networkSizeLimit, networkSize);

        std::cout << "Wireless Network:" << std::endl;
        std::cout << "Average Degree: " << network.getAverageDegree() << std::endl;
        std::cout << "Maximum Degree: " << network.getMaximumDegree() << std::endl;

        std::vector<int> path = network.compassRouting(source, destination);
        if (path.empty()) {
            std::cout << "No valid path found from source to destination." << std::endl;
        }
        else {
            std::cout << "Path from " << source << " to " << destination << ": ";
            for (int vertex : path) {
                std::cout << vertex << " ";
            }
            std::cout << std::endl;
        }
    }
};

//int main() {
//    Experiments::run();
//    return 0;
//}

int main() {
    // Set the parameters for the experiments
    int numNetworks = 1;
    int minNodes = 0;
    int maxNodes = 1000;
    float gridSize = 10.0;

    // Generate and analyze wireless networks
    for (int n = minNodes; n <= maxNodes; n += 1) {
        // Generate a wireless network
        WirelessNetwork network;
        network.generate(gridSize, n);

        // Report the average and maximum degree of the network
        std::cout << "Average and minmum degree of the network with out toplogy"<<std::endl;
        std::cout << "Network with " << n << " nodes:" << std::endl;
        std::cout << "Average Degree: " << network.getAverageDegree() << std::endl;
        std::cout << "Maximum Degree: " << network.getMaximumDegree() << std::endl;

        // Perform topology control on the network
        network.topologyControl();

        // Report the average and maximum degree of the modified network
        std::cout << "After toplogyControl algorithm"<<std::endl;
        std::cout << "Modified Network with " << n << " nodes:" << std::endl;
        std::cout << "Average Degree: " << network.getAverageDegree() << std::endl;
        std::cout << "Maximum Degree: " << network.getMaximumDegree() << std::endl;

        
    }

     //Run compass routing experiments
    int source = 0;
    int destination = 1;
    WirelessNetwork network = Loader::generateNetwork(gridSize, 1000);

    for (int i = 0; i < 10; i++) {
        std::vector<int> path = network.compassRouting(source, destination);

        std::cout << "Compass Routing Experiment " << i + 1 << std::endl;
        if (path.empty()) {
            std::cout << "No valid path found from source to destination." << std::endl;
        }
        else {
            std::cout << "Path from " << source << " to " << destination << ": ";
            for (int vertex : path) {
                std::cout << vertex << " ";
            }
            std::cout << std::endl;
        }
    }

    // Perform topology control on the network
    network.topologyControl();

    // Run compass routing experiments on the modified network
    for (int i = 0; i < 10; i++) {
        std::vector<int> path = network.compassRouting(source, destination);

        std::cout << "Compass Routing Experiment (Modified Network) " << i + 1 << std::endl;
        if (path.empty()) {
            std::cout << "No valid path found from source to destination." << std::endl;
        }
        else {
            std::cout << "Path from " << source << " to " << destination << ": ";
            for (int vertex : path) {
                std::cout << vertex << " ";
            }
            std::cout << std::endl;
        }
    }


    std::cout <<std:: endl<<std::endl;
   // Experiments::run();


    return 0;
}


