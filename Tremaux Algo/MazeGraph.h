#ifndef MAZE_GRAPH_H
#define MAZE_GRAPH_H

#include <Arduino.h>

// ============================================================
//                       MAZE GRAPH
// ============================================================

#define MAX_NODES 40
#define MAX_EDGES 80

// Absolute directions
enum AbsoluteDirection
{
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};

// ============================================================
//                           EDGE
// ============================================================

struct MazeEdge
{
    int id;

    int nodeA;
    int nodeB;

    // Direction when leaving nodeA
    int directionA;

    // Direction when leaving nodeB
    int directionB;

    // Trémaux edge count:
    // 0 = never traversed
    // 1 = traversed once
    // 2 = traversed twice
    uint8_t visits;

    bool active;
};

// ============================================================
//                           NODE
// ============================================================

struct MazeNode
{
    int id;

    // Bit mask:
    // bit 0 = NORTH
    // bit 1 = EAST
    // bit 2 = SOUTH
    // bit 3 = WEST
    uint8_t exits;

    bool goal;
    bool active;
};

// ============================================================
//                         MAZE GRAPH
// ============================================================

class MazeGraph
{

private:
    MazeNode nodes[MAX_NODES];
    MazeEdge edges[MAX_EDGES];

    int nodeCount;
    int edgeCount;

    int currentNode;
    int currentHeading;

    // --------------------------------------------------------
    // Find edge leaving a node in a particular direction
    // --------------------------------------------------------

    int findEdge(int node, int direction)
    {

        for (int i = 0; i < edgeCount; i++)
        {

            if (!edges[i].active)
                continue;

            if (edges[i].nodeA == node &&
                edges[i].directionA == direction)
            {

                return i;
            }

            if (edges[i].nodeB == node &&
                edges[i].directionB == direction)
            {

                return i;
            }
        }

        return -1;
    }

public:
    // ========================================================
    // Constructor
    // ========================================================

    MazeGraph()
    {

        nodeCount = 0;
        edgeCount = 0;

        currentNode = -1;
        currentHeading = NORTH;

        for (int i = 0; i < MAX_NODES; i++)
        {
            nodes[i].active = false;
        }

        for (int i = 0; i < MAX_EDGES; i++)
        {
            edges[i].active = false;
        }
    }

    // ========================================================
    // Create a new junction/node
    // ========================================================

    int createNode(uint8_t exits)
    {

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

    // ========================================================
    // Initialize graph at starting point
    // ========================================================

    int initializeStart(uint8_t exits, int heading)
    {

        int start = createNode(exits);

        if (start < 0)
            return -1;

        currentNode = start;
        currentHeading = heading;

        return start;
    }

    // ========================================================
    // Heading
    // ========================================================

    void setHeading(int heading)
    {

        currentHeading = heading % 4;

        if (currentHeading < 0)
            currentHeading += 4;
    }

    int getHeading()
    {
        return currentHeading;
    }

    int getCurrentNode()
    {
        return currentNode;
    }

    // ========================================================
    // Relative → Absolute direction
    //
    // relative:
    // 0 = LEFT
    // 1 = STRAIGHT
    // 2 = RIGHT
    // 3 = BACK
    // ========================================================

    int relativeToAbsolute(int relative)
    {

        if (relative == 0)
            return (currentHeading + 3) % 4;

        if (relative == 1)
            return currentHeading;

        if (relative == 2)
            return (currentHeading + 1) % 4;

        return (currentHeading + 2) % 4;
    }

    // ========================================================
    // Create edge between two nodes
    // ========================================================

    int createEdge(
        int nodeA,
        int directionA,
        int nodeB,
        int directionB)
    {

        if (nodeA < 0 || nodeA >= nodeCount)
            return -1;

        if (nodeB < 0 || nodeB >= nodeCount)
            return -1;

        if (edgeCount >= MAX_EDGES)
            return -1;

        // Do not create duplicate edge
        if (findEdge(nodeA, directionA) >= 0)
            return findEdge(nodeA, directionA);

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

    // ========================================================
    // Connect current node to a NEW node
    // ========================================================

    int connectNewNode(uint8_t exits, int directionFromCurrent)
    {

        if (currentNode < 0)
            return -1;

        int newNode = createNode(exits);

        if (newNode < 0)
            return -1;

        int directionAtNewNode =
            (directionFromCurrent + 2) % 4;

        return createEdge(
            currentNode,
            directionFromCurrent,
            newNode,
            directionAtNewNode);
    }

    // ========================================================
    // Find edge
    // ========================================================

    int getEdge(int node, int direction)
    {

        return findEdge(node, direction);
    }

    // ========================================================
    // Get edge visit count
    // ========================================================

    int getVisits(int edgeID)
    {

        if (edgeID < 0 || edgeID >= edgeCount)
            return 99;

        if (!edges[edgeID].active)
            return 99;

        return edges[edgeID].visits;
    }

    // ========================================================
    // Traverse an edge
    // ========================================================

    void traverseEdge(int edgeID)
    {

        if (edgeID < 0 || edgeID >= edgeCount)
            return;

        if (!edges[edgeID].active)
            return;

        // Trémaux maximum
        if (edges[edgeID].visits < 2)
            edges[edgeID].visits++;

        // Move from A → B
        if (edges[edgeID].nodeA == currentNode)
        {

            currentNode = edges[edgeID].nodeB;

            currentHeading =
                edges[edgeID].directionB;
        }

        // Move from B → A
        else if (edges[edgeID].nodeB == currentNode)
        {

            currentNode = edges[edgeID].nodeA;

            currentHeading =
                edges[edgeID].directionA;
        }
    }

    // ========================================================
    // Get the node at the other end of an edge
    // ========================================================

    int otherNode(int edgeID, int node)
    {

        if (edgeID < 0 || edgeID >= edgeCount)
            return -1;

        if (edges[edgeID].nodeA == node)
            return edges[edgeID].nodeB;

        if (edges[edgeID].nodeB == node)
            return edges[edgeID].nodeA;

        return -1;
    }

    // ========================================================
    // Mark current node as goal
    // ========================================================

    void markGoal()
    {

        if (currentNode >= 0)
            nodes[currentNode].goal = true;
    }

    // ========================================================
    // Check whether current node is goal
    // ========================================================

    bool isGoal(int node)
    {

        if (node < 0 || node >= nodeCount)
            return false;

        return nodes[node].goal;
    }

    // ========================================================
    // Get an unvisited edge from a node
    // ========================================================

    int getUnvisitedEdge(int node)
    {

        for (int i = 0; i < edgeCount; i++)
        {

            if (!edges[i].active)
                continue;

            if (edges[i].visits != 0)
                continue;

            if (edges[i].nodeA == node ||
                edges[i].nodeB == node)
            {

                return i;
            }
        }

        return -1;
    }

    // ========================================================
    // Get an edge that has been visited once
    // Useful for Trémaux backtracking
    // ========================================================

    int getOnceVisitedEdge(int node)
    {

        for (int i = 0; i < edgeCount; i++)
        {

            if (!edges[i].active)
                continue;

            if (edges[i].visits != 1)
                continue;

            if (edges[i].nodeA == node ||
                edges[i].nodeB == node)
            {

                return i;
            }
        }

        return -1;
    }

    // ========================================================
    // Graph information
    // ========================================================

    int getNodeCount()
    {
        return nodeCount;
    }

    int getEdgeCount()
    {
        return edgeCount;
    }

    MazeNode getNode(int id)
    {

        if (id < 0 || id >= nodeCount)
        {
            MazeNode empty;
            empty.id = -1;
            empty.exits = 0;
            empty.goal = false;
            empty.active = false;
            return empty;
        }

        return nodes[id];
    }

    MazeEdge getEdgeInfo(int id)
    {

        if (id < 0 || id >= edgeCount)
        {
            MazeEdge empty;
            empty.id = -1;
            empty.nodeA = -1;
            empty.nodeB = -1;
            empty.directionA = -1;
            empty.directionB = -1;
            empty.visits = 99;
            empty.active = false;
            return empty;
        }

        return edges[id];
    }

    // ========================================================
    // Debug
    // ========================================================

    void printGraph()
    {

        Serial.println();
        Serial.println("========== MAZE GRAPH ==========");

        Serial.print("Nodes: ");
        Serial.println(nodeCount);

        Serial.print("Edges: ");
        Serial.println(edgeCount);

        Serial.println();

        for (int i = 0; i < nodeCount; i++)
        {

            Serial.print("NODE ");
            Serial.print(i);

            Serial.print(" exits=");
            Serial.print(nodes[i].exits, BIN);

            Serial.print(" goal=");
            Serial.println(nodes[i].goal ? "YES" : "NO");
        }

        Serial.println();

        for (int i = 0; i < edgeCount; i++)
        {

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
};

#endif
