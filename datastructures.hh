
#ifndef DATASTRUCTURES_HH
#define DATASTRUCTURES_HH

#include <string>
#include <vector>
#include <tuple>
#include <utility>
#include <limits>
#include <functional>
#include <exception>
#include <deque>

//-------------BEGIN OF CODE NOT WRITTEN BY ME------------------

// Types for IDs
using TownID = std::string;
using Name = std::string;

// Return values for cases where required thing was not found
TownID const NO_TOWNID = "----------";

// Return value for cases where integer values were not found
int const NO_VALUE = std::numeric_limits<int>::min();

// Return value for cases where name values were not found
Name const NO_NAME = "!!NO_NAME!!";

// Type for a coordinate (x, y)
struct Coord
{
    int x = NO_VALUE;
    int y = NO_VALUE;
};

// Example: Defining == and hash function for Coord so that it can be used
// as key for std::unordered_map/set, if needed
inline bool operator==(Coord c1, Coord c2) { return c1.x == c2.x && c1.y == c2.y; }
inline bool operator!=(Coord c1, Coord c2) { return !(c1==c2); } // Not strictly necessary

struct CoordHash
{
    std::size_t operator()(Coord xy) const
    {
        auto hasher = std::hash<int>();
        auto xhash = hasher(xy.x);
        auto yhash = hasher(xy.y);
        // Combine hash values (magic!)
        return xhash ^ (yhash + 0x9e3779b9 + (xhash << 6) + (xhash >> 2));
    }
};

// Example: Defining < for Coord so that it can be used
// as key for std::map/set
inline bool operator<(Coord c1, Coord c2)
{
    if (c1.y < c2.y) { return true; }
    else if (c2.y < c1.y) { return false; }
    else { return c1.x < c2.x; }
}

// Return value for cases where coordinates were not found
Coord const NO_COORD = {NO_VALUE, NO_VALUE};

// Type for a distance (in metres)
using Distance = int;

// Return value for cases where Distance is unknown
Distance const NO_DISTANCE = NO_VALUE;

// This exception class is there just so that the user interface can notify
// about operations which are not (yet) implemented
class NotImplemented : public std::exception
{
public:
    NotImplemented() : msg_{} {}
    explicit NotImplemented(std::string const& msg) : msg_{msg + " not implemented"} {}

    virtual const char* what() const noexcept override
    {
        return msg_.c_str();
    }
private:
    std::string msg_;
};

//-------------END OF CODE NOT WRITTEN BY ME------------------

// This is the class you are supposed to implement

class Datastructures
{
public:
    Datastructures();
    ~Datastructures();

    // Estimate of performance: O(1)
    // Short rationale for estimate: .size() is constant in std::map
    unsigned int town_count();

    // Estimate of performance: O(n)
    // Short rationale for estimate: .clear() is linear with the size of the map
    void clear_all();

    // Estimate of performance: θ(1) and O(n)
    // Short rationale for estimate: Because of the .find() time complexity increases linearly with map size if the added place doesn't exist.
    bool add_town(TownID id, Name const& name, Coord coord, int tax);

    // Estimate of performance: θ(1) and O(n)
    // Short rationale for estimate: [] operator and .find have the same performance
    Name get_town_name(TownID id);

    // Estimate of performance: θ(1) and O(n)
    // Short rationale for estimate: [] operator and .find have the same performance
    Coord get_town_coordinates(TownID id);

    // Estimate of performance: θ(1) and O(n)
    // Short rationale for estimate: [] operator and .find have the same performance
    int get_town_tax(TownID id);

    // Estimate of performance: O(n)
    // Short rationale for estimate: Because of the for-loop time complexity increases linearly with map size.
    std::vector<TownID> all_towns();

    // Estimate of performance: O(n)
    // Short rationale for estimate: Because of the for-loop time complexity increases linearly with map size.
    std::vector<TownID> find_towns(Name const& name);

    // Estimate of performance: θ(1) and O(n)
    // Short rationale for estimate: Because of the .find() time complexity increases linearly with map size if the added place doesn't exist.
    bool change_town_name(TownID id, Name const& newname);

    // Estimate of performance: θ(nlog(n))
    // Short rationale for estimate: .sort() has average performance of θ(nlog(n)) which is bigger than the θ(n) complexies in the for-loops.
    std::vector<TownID> towns_alphabetically();

    // Estimate of performance:  θ(nlog(n))
    // Short rationale for estimate: .sort() has average performance of θ(nlog(n)) which is bigger than the θ(n) complexies in the for-loops.
    std::vector<TownID> towns_distance_increasing();

    // Estimate of performance: O(1)
    // Short rationale for estimate: No loops or operators
    TownID min_distance();

    // Estimate of performance: O(1)
    // Short rationale for estimate: No loops or operators
    TownID max_distance();

    // Estimate of performance: θ(1) and O(n)
    // Short rationale for estimate: Because of the .find() time complexity increases linearly with map size if the added place doesn't exist.
    bool add_vassalship(TownID vassalid, TownID masterid);

    // Estimate of performance: θ(1) and O(n)
    // Short rationale for estimate: Because of the .find() time complexity increases linearly with map size if the added place doesn't exist.
    std::vector<TownID> get_town_vassals(TownID id);

    // Estimate of performance: θ(1) and O(n)
    // Short rationale for estimate: Because of the .find() time complexity increases linearly with map size if the added place doesn't exist.
    std::vector<TownID> taxer_path(TownID id);

    // Non-compulsory phase 1 operations

    // Estimate of performance: θ(1) and O(n)
    // Short rationale for estimate: Because of the .find() time complexity increases linearly with map size if the added place doesn't exist.
    bool remove_town(TownID id);

    // Estimate of performance: θ(n) and O(n)
    // Short rationale for estimate: The performance linear because of the for-loop
    std::vector<TownID> towns_nearest(Coord coord);

    // Estimate of performance: θ(1) and O(n)
    // Short rationale for estimate: Because of the .find() time complexity increases linearly with map size if the added place doesn't exist.
    std::vector<TownID> longest_vassal_path(TownID id);

    // Estimate of performance: θ(1) and O(n)
    // Short rationale for estimate: Because of the .find() time complexity increases linearly with map size if the added place doesn't exist.
    int total_net_tax(TownID id);


    // Phase 2 operations

    // Estimate of performance: O(n)
    // Short rationale for estimate: Lineaarinen suhteessa unordered_mapin pituuteen
    void clear_roads();

    // Estimate of performance: O(1)
    // Short rationale for estimate: Vakio nopeuksinen
    std::vector<std::pair<TownID, TownID>> all_roads();

    // Estimate of performance: θ(1) and O(n)
    // Short rationale for estimate: .find on pahimmassa tapauksessa lineaarinen ja keskimäärin vakio
    bool add_road(TownID town1, TownID town2);

    // Estimate of performance: θ(1) and O(n)
    // Short rationale for estimate: .find on pahimmassa tapauksessa lineaarinen ja keskimäärin vakio
    std::vector<TownID> get_roads_from(TownID id);

    // Estimate of performance: O(n+k)
    // Short rationale for estimate: BFS algoritmin tehokkuus on O(n+k)
    std::vector<TownID> any_route(TownID fromid, TownID toid);

    // Non-compulsory phase 2 operations

    // Estimate of performance: O(n)
    // Short rationale for estimate: .find on vektorilla pahimmillaan lineaarinen
    bool remove_road(TownID town1, TownID town2);

    // Estimate of performance: O(n+k)
    // Short rationale for estimate: BFS algoritmin tehokkuus on O(n+k)
    std::vector<TownID> least_towns_route(TownID fromid, TownID toid);

    // Estimate of performance: O(n+k)
    // Short rationale for estimate: DFS algoritmin tehokkuus on O(n+k)
    std::vector<TownID> road_cycle_route(TownID startid);

    // Estimate of performance: O(n+k)*log*(n+k))
    // Short rationale for estimate: Dijkstran algoritmin tehokkuus on O(n+k)*log*(n+k)) ja A* on muunneltu Dijkstran algoritmi.
    // Tehokkuus ei ole O(n+k)*log*(n)), koska alkiot lisätää uudestaan priority_queuhun vaikka ne olisi jo käyty
    std::vector<TownID> shortest_route(TownID fromid, TownID toid);

    // Estimate of performance: O(N*logN)
    // Short rationale for estimate: Kruskalin algoritmin tehokkuus on wikipedian mukaan tuollainen
    Distance trim_road_network();


private:
    // Add stuff needed for your class implementation here

    // Estimate of performance: O(1)
    // Short rationale for estimate: No loops or operators
    int distance_between_points(Coord first, Coord second);

    // Estimate of performance: O(n)
    // Short rationale for estimate: std::minmax_element is linear in complexity
    void update_distance();

    // Estimate of performance: O(n)
    // Short rationale for estimate: The performance is linear because of the for-loop
    void longest_tax_path_recursive(TownID id, int depth);

    // Estimate of performance: O(n)
    // Short rationale for estimate: The performance is linear because of the for-loop
    int vassal_net_tax_recursive(TownID id);


    // Phase 2 operations

    // Estimate of performance: O(n+k)
    // Short rationale for estimate: DFS algoritmin tehokkuus on O(n+k)
    void depth_first_recursive(TownID start, TownID previous);

    // Estimate of performance: O(n+k)*log*(n+k))
    // Short rationale for estimate: Dijkstran algoritmin tehokkuus on O(n+k)*log*(n+k)) ja A* on muunneltu Dijkstran algoritmi.
    // Tehokkuus ei ole O(n+k)*log*(n)), koska alkiot lisätää uudestaan priority_queuhun vaikka ne olisi jo käyty
    std::vector<TownID> A_star(TownID start, TownID goal);

    // Estimate of performance: Theta(1)
    // Short rationale for estimate: unordered_mapissa .at() on keskimäärin vakiotehokkuuksinen
    void relax(TownID neighbour, TownID curr_town, TownID goalID);

    // Estimate of performance: θ(1)
    // Short rationale for estimate: .at on keskimäärin vakionopeuksinen
    int distance_between_towns(TownID town1, TownID town2);

    // Estimate of performance: O(n)
    // Short rationale for estimate: Lineaarinen suhteessa unordered_mapin pituuteen
    void reset_visits();

    // Estimate of performance: O(n)
    // Short rationale for estimate: Lineaarinen suhteessa vektorin pituuteen
    std::vector<TownID> get_route_vector(TownID startID, TownID goalID);



    struct Single_town
    {
        Name name_;
        Coord coordinates_;
        TownID parent_town = NO_TOWNID;
        std::vector<TownID> vector_of_vassals;
        int tax_ = NO_VALUE;
        int distance_;


        std::vector<TownID> vector_roads_to;
        bool visited = false;
        TownID reached_from = NO_TOWNID;
        int cost;
        int estimated_cost;
    };

    std::unordered_map<TownID,Single_town> map_of_towns;
    std::vector<std::pair<TownID,TownID>> vector_all_roads;

    TownID closest_town = NO_TOWNID;
    int closest_dist = NO_DISTANCE;
    TownID furthest_town = NO_TOWNID;
    int furthest_dist = NO_DISTANCE;

    int curr_longest_depth;
    TownID curr_base_vassal;

    TownID start_town;
    std::vector<TownID> loop_vector;

    int negative_infinity = -1 * std::numeric_limits<int>::max();


};

#endif // DATASTRUCTURES_HH
