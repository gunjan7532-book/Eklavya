#ifndef MAZE_GRAPH_H
#define MAZE_GRAPH_H
// ============================================================
// MAZE GRAPH FOR TREMAUX
// ============================================================
#define MAX_NODES 40
#define MAX_EDGES 80
// Absolute directions
// 0 = NORTH
// 1 = EAST
// 2 = SOUTH
// 3 = WEST
enum Direction {
    NORTH = 0,
    EAST  = 1,
    SOUTH = 2,
    WEST  = 3
};
// ------------------------------------------------------------
// Edge
// ------------------------------------------------------------
struct MazeEdge {
    int id;
    int nodeA;
    int nodeB;
    int directionA;
    int directionB;
    // Trémaux:
    // 0 = never traversed
    // 1 = traversed once
    // 2 = traversed twice
    int visits;
    bool active;
};

// ------------------------------------------------------------
// Node / Junction
// ------------------------------------------------------------
struct MazeNode {
    int id;
    // Which absolute directions have a branch?
    // Bit 0 = NORTH
    // Bit 1 = EAST
    // Bit 2 = SOUTH
    // Bit 3 = WEST
    uint8_t exits;
    bool goal;
    bool active;
};

// ------------------------------------------------------------
// Graph
// ------------------------------------------------------------
class MazeGraph {
private:
    MazeNode nodes[MAX_NODES];
    MazeEdge edges[MAX_EDGES];
    int nodeCount;
    int edgeCount;
    int currentNode;
    int currentHeading;
    // --------------------------------------------------------
    // Internal helpers
    // --------------------------------------------------------
    int findEdge(int node, int direction) {
        for (int i = 0; i < edgeCount; i++) {
            if (!edges[i].active)
                continue;
            if (edges[i].nodeA == node &&
                edges[i].directionA == direction) {
                return i;
            }
            if (edges[i].nodeB == node &&
                edges[i].directionB == direction) {
                return i;
            }
        }
        return -1;
    }
public:
    // --------------------------------------------------------
    // Constructor
    // --------------------------------------------------------
    MazeGraph() {
        nodeCount = 0;
        edgeCount = 0;
        currentNode = -1;
        currentHeading = NORTH;
        for (int i = 0; i < MAX_NODES; i++)
            nodes[i].active = false;
        for (int i = 0; i < MAX_EDGES; i++)
            edges[i].active = false;
    }

    // --------------------------------------------------------
    // Create a node
    // --------------------------------------------------------
    int createNode(uint8_t exits) {
        if (nodeCount >= MAX_NODES)
            return -1;
        int id = nodeCount;
        nodes[id].id = id;
        nodes[id].exits = exits;
        nodes[id].goal = false;
        nodes[id].active = true;
        nodeCount++;
        return id;
    }

    // --------------------------------------------------------
    // Find node with same junction signature
    // --------------------------------------------------------
    int findNodeBySignature(uint8_t exits) {
        for (int i = 0; i < nodeCount; i++) {
            if (!nodes[i].active)
                continue;
            if (nodes[i].exits == exits)
                return i;
        }
        return -1;
    }

    // --------------------------------------------------------
    // Add an edge
    // --------------------------------------------------------

    int createEdge(
        int nodeA,
        int directionA,
        int nodeB,
        int directionB
    ) {
        if (edgeCount >= MAX_EDGES)
            return -1;
        int id = edgeCount;
        edges[id].id = id;
        edges[id].nodeA = nodeA;
        edges[id].nodeB = nodeB;
        edges[id].directionA = directionA;
        edges[id].directionB = directionB;
        edges[id].visits = 0;
        edges[id].active = true;
        edgeCount++;
        return id;
    }

    // --------------------------------------------------------
    // Start graph
    // --------------------------------------------------------

    int initializeStart(uint8_t exits, int heading) {
        int start = createNode(exits);
        if (start < 0)
            return -1;
        currentNode = start;
        currentHeading = heading;
        return start;
    }

    // --------------------------------------------------------
    // Set heading
    // --------------------------------------------------------

    void setHeading(int heading) {
        currentHeading = heading;
        if (currentHeading < 0)
            currentHeading += 4;
        if (currentHeading >= 4)
            currentHeading -= 4;
    }
    int getHeading() {
        return currentHeading;
    }
    int getCurrentNode() {
        return currentNode;
    }

    // --------------------------------------------------------
    // Convert relative direction to absolute direction
    //
    // relative:
    // 0 = left
    // 1 = straight
    // 2 = right
    // 3 = back
    // --------------------------------------------------------
    int relativeToAbsolute(int relative) {
        if (relative == 0)
            return (currentHeading + 3) % 4;
        if (relative == 1)
            return currentHeading;
        if (relative == 2)
            return (currentHeading + 1) % 4;
        // back
        return (currentHeading + 2) % 4;
    }

    // --------------------------------------------------------
    // Check whether edge exists
    // --------------------------------------------------------
    int getEdge(int node, int direction) {
        return findEdge(node, direction);
    }

// --------------------------------------------------------
    // Get edge visit count
    // --------------------------------------------------------
    int getVisits(int edgeID) {
        if (edgeID < 0 || edgeID >= edgeCount)
            return 99;
        return edges[edgeID].visits;
    }

    // --------------------------------------------------------
    // Traverse edge
    // --------------------------------------------------------
    void traverseEdge(int edgeID) {
        if (edgeID < 0 || edgeID >= edgeCount)
            return;
        edges[edgeID].visits++;
        if (edges[edgeID].visits > 2)
            edges[edgeID].visits = 2;
        if (edges[edgeID].nodeA == currentNode) {
            currentNode = edges[edgeID].nodeB;
            currentHeading = edges[edgeID].directionB;
        } else {
            currentNode = edges[edgeID].nodeA;
            currentHeading = edges[edgeID].directionA;
        }
    }

    // --------------------------------------------------------
    // Connect current node to another node
    // --------------------------------------------------------
    int connectToNode(
        int targetNode,
        int directionFromCurrent
    ) {
        int existing = findEdge(
            currentNode,
            directionFromCurrent
        );
        if (existing >= 0)
            return existing;
        int targetDirection =
            (directionFromCurrent + 2) % 4;
        return createEdge(
            currentNode,
            directionFromCurrent,
            targetNode,
            targetDirection
        );
    }

    // --------------------------------------------------------
    // Get the node reached by an edge
    // --------------------------------------------------------
    int otherNode(int edgeID, int node) {
        if (edges[edgeID].nodeA == node)
            return edges[edgeID].nodeB;
        return edges[edgeID].nodeA;
    }

    // --------------------------------------------------------
    // Mark goal
    // --------------------------------------------------------
    void markGoal() {
        if (currentNode >= 0)
            nodes[currentNode].goal = true;
    }

    // --------------------------------------------------------
    // Debug print
    // --------------------------------------------------------
    void printGraph() {
        Serial.println();
        Serial.println("========== MAZE GRAPH ==========");
        Serial.print("Nodes: ");
        Serial.println(nodeCount);
        Serial.print("Edges: ");
        Serial.println(edgeCount);
        Serial.println();
        for (int i = 0; i < nodeCount; i++) {
            Serial.print("NODE ");
            Serial.print(i);
            Serial.print(" exits=");
            Serial.println(nodes[i].exits, BIN);
        }
        Serial.println();
        for (int i = 0; i < edgeCount; i++) {
            Serial.print("EDGE ");
            Serial.print(i);
            Serial.print(": ");
            Serial.print(edges[i].nodeA);
            Serial.print(" --(");
            Serial.print(edges[i].directionA);
            Serial.print("/");
            Serial.print(edges[i].directionB);
            Serial.print(")-- ");
            Serial.print(edges[i].nodeB);
            Serial.print(" visits=");
            Serial.println(edges[i].visits);
        }
        Serial.println("================================");
        Serial.println();
    }

    // --------------------------------------------------------
    // Accessors
    // --------------------------------------------------------
    int getNodeCount() {
        return nodeCount;
    }
    int getEdgeCount() {
        return edgeCount;
    }
    MazeNode getNode(int id) {
        return nodes[id];
    }
    MazeEdge getEdgeInfo(int id) {
        return edges[id];
    }
};

#endif
