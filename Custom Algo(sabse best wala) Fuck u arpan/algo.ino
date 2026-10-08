
#define MAZE_WIDTH 10
#define MAZE_HEIGHT 10

#define TOTAL_CELLS (MAZE_WIDTH * MAZE_HEIGHT)

#define TOTAL_STATES (TOTAL_CELLS * 4)

// ================================================================
//                        DIRECTIONS
// ================================================================

enum Direction
{
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};

// ================================================================
//                        MAZE CELL
// ================================================================

struct Cell
{
    /*
       wall[0] = North
       wall[1] = East
       wall[2] = South
       wall[3] = West
    */

    bool wall[4];

    bool visited;
};

// Our maze

Cell maze[MAZE_WIDTH][MAZE_HEIGHT];

// ================================================================
//                    FLOOD FILL VARIABLES
// ================================================================

int floodValue[MAZE_WIDTH][MAZE_HEIGHT];

// ================================================================
//              ROBOT SPEED CALIBRATION
// ================================================================

/*
   THESE ARE THE IMPORTANT VALUES.

   Replace these with measurements from YOUR robot.

   All times are in milliseconds.

   Example:

       straightTimePerCell = 220

   means the robot takes approximately 220 ms to travel
   one maze cell on a straight line.
*/

float straightTimePerCell = 220.0;

// Time required to perform a 90 degree LEFT turn

float leftTurnTime = 310.0;

// Time required to perform a 90 degree RIGHT turn

float rightTurnTime = 280.0;

// Time required to perform a 180 degree turn

float uTurnTime = 600.0;

// ================================================================
//                  OPTIONAL SPEED INFORMATION
// ================================================================

/*
   This is useful for displaying information and for later
   improvements.

   Example:

       straightSpeed = 80 cm/s

   The optimizer currently uses straightTimePerCell directly,
   which is more accurate for a fixed maze-cell distance.
*/

float straightSpeed = 80.0;

// ================================================================
//                  DIJKSTRA STATE
// ================================================================

/*
   A state contains:

       X position
       Y position
       Heading

   Example:

       (3,4,EAST)

   is different from:

       (3,4,NORTH)
*/

struct State
{
    byte x;
    byte y;
    Direction heading;
};

// ================================================================
//              DIJKSTRA ARRAYS
// ================================================================

// Best known time to reach every state

float distanceCost[TOTAL_STATES];

// Previous state.
// Used later to reconstruct the actual route.

int previousState[TOTAL_STATES];

// Has this state already been finalized?

bool stateVisited[TOTAL_STATES];

// ================================================================
//                    ROUTE STORAGE
// ================================================================

#define MAX_ROUTE_LENGTH 100

int route[MAX_ROUTE_LENGTH];

int routeLength = 0;

// ================================================================
//                STATE ID CONVERSION
// ================================================================

/*
   Converts:

       x
       y
       heading

   into one integer.

   Example:

       stateID(2,3,EAST)

   gives one unique number.
*/

int stateID(
    int x,
    int y,
    Direction heading)
{
    return ((y * MAZE_WIDTH) + x) * 4 + heading;
}

// ---------------------------------------------------------------
// Convert a state ID back into X,Y,Heading
// ---------------------------------------------------------------

State getState(int id)
{
    State s;

    // Heading is the remainder after dividing by 4

    s.heading = (Direction)(id % 4);

    // Remove heading

    int cellNumber = id / 4;

    // Recover X and Y

    s.x = cellNumber % MAZE_WIDTH;

    s.y = cellNumber / MAZE_WIDTH;

    return s;
}

// ================================================================
//                    INITIALIZE MAZE
// ================================================================

void initializeMaze()
{
    for (int x = 0; x < MAZE_WIDTH; x++)
    {
        for (int y = 0; y < MAZE_HEIGHT; y++)
        {
            maze[x][y].visited = false;

            for (int d = 0; d < 4; d++)
            {
                maze[x][y].wall[d] = false;
            }
        }
    }

    /*
       Add outer boundary walls.

               NORTH
                 ↑
          ┌──────────────┐
          │              │
   WEST ← │     MAZE     │ → EAST
          │              │
          └──────────────┘
                 ↓
               SOUTH
    */

    // Bottom wall

    for (int x = 0; x < MAZE_WIDTH; x++)
    {
        maze[x][0].wall[SOUTH] = true;
    }

    // Top wall

    for (int x = 0; x < MAZE_WIDTH; x++)
    {
        maze[x][MAZE_HEIGHT - 1].wall[NORTH] = true;
    }

    // Left wall

    for (int y = 0; y < MAZE_HEIGHT; y++)
    {
        maze[0][y].wall[WEST] = true;
    }

    // Right wall

    for (int y = 0; y < MAZE_HEIGHT; y++)
    {
        maze[MAZE_WIDTH - 1][y].wall[EAST] = true;
    }
}

// ================================================================
//                    ADD A WALL
// ================================================================

/*
   IMPORTANT:

   When we add a wall between two cells, BOTH cells need to know
   about that wall.

   Example:

       Cell A | Cell B

   If the EAST wall of A exists,

       WEST wall of B

   must also exist.
*/

void addWall(
    int x,
    int y,
    Direction direction)
{
    // Make sure cell is valid

    if (
        x < 0 ||
        x >= MAZE_WIDTH ||
        y < 0 ||
        y >= MAZE_HEIGHT)
    {
        return;
    }

    // Add wall to current cell

    maze[x][y].wall[direction] = true;

    // Find neighboring cell

    int nx = x;
    int ny = y;

    if (direction == NORTH)
        ny++;

    else if (direction == EAST)
        nx++;

    else if (direction == SOUTH)
        ny--;

    else if (direction == WEST)
        nx--;

    // Make sure neighbor exists

    if (
        nx < 0 ||
        nx >= MAZE_WIDTH ||
        ny < 0 ||
        ny >= MAZE_HEIGHT)
    {
        return;
    }

    // Opposite wall

    Direction opposite =
        (Direction)((direction + 2) % 4);

    maze[nx][ny].wall[opposite] = true;
}

// ================================================================
//                    FLOOD FILL
// ================================================================

/*
   Flood Fill answers:

       "How many CELLS away is this cell from the goal?"

   It does NOT know anything about robot speed.

   That's why we use it for:

       maze exploration
       distance information
       navigation assistance

   Dijkstra will later handle actual TIME optimization.
*/

void floodFill(
    int goalX,
    int goalY)
{
    // --------------------------------------------------------------
    // Step 1:
    // Set every cell to a very large distance.
    // --------------------------------------------------------------

    for (int x = 0; x < MAZE_WIDTH; x++)
    {
        for (int y = 0; y < MAZE_HEIGHT; y++)
        {
            floodValue[x][y] = 999;
        }
    }

    // Goal has distance 0

    floodValue[goalX][goalY] = 0;

    // --------------------------------------------------------------
    // Step 2:
    // Repeatedly improve neighboring cells.
    // --------------------------------------------------------------

    bool changed = true;

    while (changed)
    {
        changed = false;

        for (int x = 0; x < MAZE_WIDTH; x++)
        {
            for (int y = 0; y < MAZE_HEIGHT; y++)
            {
                int bestValue = floodValue[x][y];

                // ========================================================
                // NORTH
                // ========================================================

                if (
                    y < MAZE_HEIGHT - 1 &&
                    !maze[x][y].wall[NORTH])
                {
                    bestValue = min(
                        bestValue,
                        floodValue[x][y + 1] + 1);
                }

                // ========================================================
                // EAST
                // ========================================================

                if (
                    x < MAZE_WIDTH - 1 &&
                    !maze[x][y].wall[EAST])
                {
                    bestValue = min(
                        bestValue,
                        floodValue[x + 1][y] + 1);
                }

                // ========================================================
                // SOUTH
                // ========================================================

                if (
                    y > 0 &&
                    !maze[x][y].wall[SOUTH])
                {
                    bestValue = min(
                        bestValue,
                        floodValue[x][y - 1] + 1);
                }

                // ========================================================
                // WEST
                // ========================================================

                if (
                    x > 0 &&
                    !maze[x][y].wall[WEST])
                {
                    bestValue = min(
                        bestValue,
                        floodValue[x - 1][y] + 1);
                }

                // Update if a better value was found

                if (bestValue != floodValue[x][y])
                {
                    floodValue[x][y] = bestValue;

                    changed = true;
                }
            }
        }
    }
}

// ================================================================
//                GET TURN TYPE / TURN COST
// ================================================================

/*
   Compare current heading with desired movement direction.

   Example:

       Current = EAST
       Next    = NORTH

   That's a LEFT turn.

   We return the actual measured time.
*/

float getTurnTime(
    Direction currentHeading,
    Direction nextDirection)
{
    int difference =
        (nextDirection - currentHeading + 4) % 4;

    // --------------------------------------------------------------
    // Straight
    // --------------------------------------------------------------

    if (difference == 0)
    {
        return 0;
    }

    // --------------------------------------------------------------
    // Right
    // --------------------------------------------------------------

    if (difference == 1)
    {
        return rightTurnTime;
    }

    // --------------------------------------------------------------
    // U-turn
    // --------------------------------------------------------------

    if (difference == 2)
    {
        return uTurnTime;
    }

    // --------------------------------------------------------------
    // Left
    // --------------------------------------------------------------

    return leftTurnTime;
}

// ================================================================
//              CALCULATE MOVEMENT COST
// ================================================================

/*
   This is the HEART of the optimizer.

   Every move has:

       straight travel time

   PLUS:

       turn time

   Therefore:

       cost = straightTime + turnTime
*/

float movementCost(
    Direction currentHeading,
    Direction nextDirection)
{
    float cost = 0;

    // --------------------------------------------------------------
    // Time to travel one cell
    // --------------------------------------------------------------

    cost += straightTimePerCell;

    // --------------------------------------------------------------
    // Time to change direction
    // --------------------------------------------------------------

    cost += getTurnTime(
        currentHeading,
        nextDirection);

    return cost;
}

// ================================================================
//              CHECK WHETHER MOVE IS POSSIBLE
// ================================================================

bool canMove(
    int x,
    int y,
    Direction direction)
{
    // Outside maze

    if (
        x < 0 ||
        x >= MAZE_WIDTH ||
        y < 0 ||
        y >= MAZE_HEIGHT)
    {
        return false;
    }

    // Wall exists

    if (maze[x][y].wall[direction])
    {
        return false;
    }

    return true;
}

// ================================================================
//                    GET NEXT CELL
// ================================================================

void getNextCell(
    int x,
    int y,
    Direction direction,
    int &newX,
    int &newY)
{
    newX = x;
    newY = y;

    if (direction == NORTH)
        newY++;

    else if (direction == EAST)
        newX++;

    else if (direction == SOUTH)
        newY--;

    else if (direction == WEST)
        newX--;
}

// ================================================================
//              FIND CHEAPEST UNVISITED STATE
// ================================================================

int getCheapestState()
{
    float bestCost = 99999999;

    int bestID = -1;

    for (int i = 0; i < TOTAL_STATES; i++)
    {
        if (
            !stateVisited[i] &&
            distanceCost[i] < bestCost)
        {
            bestCost = distanceCost[i];

            bestID = i;
        }
    }

    return bestID;
}

// ================================================================
//                    DIJKSTRA
// ================================================================

void dijkstra(
    int startX,
    int startY,
    Direction startHeading,

    int goalX,
    int goalY)
{
    Serial.println();
    Serial.println(
        "==============================");

    Serial.println(
        "TIME-WEIGHTED DIJKSTRA");

    Serial.println(
        "==============================");

    // --------------------------------------------------------------
    // Initialize arrays
    // --------------------------------------------------------------

    for (int i = 0; i < TOTAL_STATES; i++)
    {
        distanceCost[i] = 99999999;

        previousState[i] = -1;

        stateVisited[i] = false;
    }

    // --------------------------------------------------------------
    // Starting state
    // --------------------------------------------------------------

    int startID =
        stateID(
            startX,
            startY,
            startHeading);

    distanceCost[startID] = 0;

    // --------------------------------------------------------------
    // Main Dijkstra loop
    // --------------------------------------------------------------

    for (
        int iteration = 0;
        iteration < TOTAL_STATES;
        iteration++)
    {
        // Find cheapest state

        int currentID =
            getCheapestState();

        // No more reachable states

        if (currentID == -1)
        {
            break;
        }

        // Mark visited

        stateVisited[currentID] = true;

        // Get state information

        State current =
            getState(currentID);

        // ------------------------------------------------------------
        // Stop when we reach the goal.
        //
        // We can't stop immediately if we wanted the cheapest GOAL
        // heading specifically, so we'll continue until the cheapest
        // goal state is finalized.
        // ------------------------------------------------------------

        if (
            current.x == goalX &&
            current.y == goalY)
        {
            /*
               Because Dijkstra always selects the smallest remaining
               cost, this is the globally cheapest goal state.
            */

            break;
        }

        // ------------------------------------------------------------
        // Try all four directions
        // ------------------------------------------------------------

        for (int d = 0; d < 4; d++)
        {
            Direction nextDirection =
                (Direction)d;

            // Is there a wall?

            if (
                !canMove(
                    current.x,
                    current.y,
                    nextDirection))
            {
                continue;
            }

            // Get next position

            int newX;
            int newY;

            getNextCell(
                current.x,
                current.y,
                nextDirection,
                newX,
                newY);

            // Safety check

            if (
                newX < 0 ||
                newX >= MAZE_WIDTH ||
                newY < 0 ||
                newY >= MAZE_HEIGHT)
            {
                continue;
            }

            // New state

            int newID =
                stateID(
                    newX,
                    newY,
                    nextDirection);

            // Cost of this movement

            float cost =
                movementCost(
                    current.heading,
                    nextDirection);

            // New total cost

            float newCost =
                distanceCost[currentID] + cost;

            // ----------------------------------------------------------
            // Relaxation
            // ----------------------------------------------------------

            if (
                newCost <
                distanceCost[newID])
            {
                distanceCost[newID] =
                    newCost;

                previousState[newID] =
                    currentID;
            }
        }
    }

    // ==============================================================
    // FIND BEST GOAL STATE
    // ==============================================================

    int bestGoalID = -1;

    float bestGoalCost = 99999999;

    for (int h = 0; h < 4; h++)
    {
        int id =
            stateID(
                goalX,
                goalY,
                (Direction)h);

        if (
            distanceCost[id] <
            bestGoalCost)
        {
            bestGoalCost =
                distanceCost[id];

            bestGoalID =
                id;
        }
    }

    // --------------------------------------------------------------
    // No route
    // --------------------------------------------------------------

    if (bestGoalID == -1)
    {
        Serial.println(
            "NO ROUTE FOUND!");

        return;
    }

    // --------------------------------------------------------------
    // Print best predicted time
    // --------------------------------------------------------------

    Serial.print(
        "BEST PREDICTED TIME: ");

    Serial.print(
        bestGoalCost / 1000.0);

    Serial.println(
        " seconds");

    // --------------------------------------------------------------
    // Reconstruct route
    // --------------------------------------------------------------

    reconstructRoute(
        bestGoalID);
}

// ================================================================
//                  RECONSTRUCT ROUTE
// ================================================================

void reconstructRoute(
    int goalID)
{
    routeLength = 0;

    int currentID = goalID;

    // Follow previous-state links backwards

    while (
        currentID != -1 &&
        routeLength < MAX_ROUTE_LENGTH)
    {
        route[routeLength] =
            currentID;

        routeLength++;

        currentID =
            previousState[currentID];
    }

    // --------------------------------------------------------------
    // Reverse the route
    // --------------------------------------------------------------

    for (
        int i = 0;
        i < routeLength / 2;
        i++)
    {
        int temp =
            route[i];

        route[i] =
            route[routeLength - 1 - i];

        route[routeLength - 1 - i] = temp;
    }

    // --------------------------------------------------------------
    // Print route
    // --------------------------------------------------------------

    printRoute();
}

// ================================================================
//                      PRINT ROUTE
// ================================================================

void printRoute()
{
    Serial.println();

    Serial.println(
        "===== OPTIMAL ROUTE =====");

    for (
        int i = 0;
        i < routeLength;
        i++)
    {
        State s =
            getState(route[i]);

        Serial.print("(");

        Serial.print(s.x);

        Serial.print(",");

        Serial.print(s.y);

        Serial.print(",");

        if (s.heading == NORTH)
            Serial.print("N");

        else if (s.heading == EAST)
            Serial.print("E");

        else if (s.heading == SOUTH)
            Serial.print("S");

        else
            Serial.print("W");

        Serial.print(")");

        if (i < routeLength - 1)
        {
            Serial.print(" -> ");
        }
    }

    Serial.println();
}

// ================================================================
//                  PRINT FLOOD MAP
// ================================================================

void printFloodMap()
{
    Serial.println();

    Serial.println(
        "===== FLOOD MAP =====");

    /*
       Print from NORTH row to SOUTH row so the map looks
       more natural when viewed in Serial Monitor.
    */

    for (
        int y = MAZE_HEIGHT - 1;
        y >= 0;
        y--)
    {
        for (
            int x = 0;
            x < MAZE_WIDTH;
            x++)
        {
            Serial.print(
                floodValue[x][y]);

            Serial.print("\t");
        }

        Serial.println();
    }
}

// ================================================================
//              CREATE TEST MAZE
// ================================================================

void createTestMaze()
{
    /*
       THIS IS ONLY A TEST MAZE.

       Later, the real maze walls will come from
       the robot's IR/junction detection.

       For now, we can manually create walls to test
       the algorithm.
    */

    // Example walls

    addWall(
        2,
        0,
        NORTH);

    addWall(
        2,
        1,
        NORTH);

    addWall(
        4,
        1,
        EAST);

    addWall(
        4,
        2,
        EAST);

    addWall(
        6,
        3,
        NORTH);

    addWall(
        6,
        4,
        NORTH);
}

// ================================================================
//                        SETUP
// ================================================================

void setup()
{
    Serial.begin(115200);

    Serial.println();

    Serial.println(
        "MESMERIZE ROUTE OPTIMIZER");

    // --------------------------------------------------------------
    // Initialize maze
    // --------------------------------------------------------------

    initializeMaze();

    // --------------------------------------------------------------
    // Create test maze
    // --------------------------------------------------------------

    createTestMaze();

    // --------------------------------------------------------------
    // Flood Fill
    // --------------------------------------------------------------

    floodFill(
        MAZE_WIDTH - 1,
        MAZE_HEIGHT - 1);

    // Show Flood Fill map

    printFloodMap();

    // --------------------------------------------------------------
    // Dijkstra
    // --------------------------------------------------------------

    dijkstra(
        0, // Start X
        0, // Start Y

        EAST, // Starting heading

        MAZE_WIDTH - 1, // Goal X
        MAZE_HEIGHT - 1 // Goal Y
    );
}

// ================================================================
//                         LOOP
// ================================================================

void loop()
{
    /*
       Nothing here yet.

       This program currently calculates the route ONCE.

       Later:

         Mapping run
              ↓
         Update maze
              ↓
         Flood Fill
              ↓
         Calibrate/measured values
              ↓
         Dijkstra
              ↓
         Execute route
    */
}