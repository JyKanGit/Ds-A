// Datastructures.cc


#include <queue>

#include "datastructures.hh"

#include <random>

#include <cmath>


// Modify the code below to implement the functionality of the class.
// Also remove comments from the parameter names when you implement
// an operation (Commenting out parameter name prevents compiler from
// warning about unused parameters on operations you haven't yet implemented.)

Datastructures::Datastructures()
{
    // Write any initialization you need here
}

Datastructures::~Datastructures()
{
    // Write any cleanup you need here
}

unsigned int Datastructures::town_count()
{

    return map_of_towns.size();

}

void Datastructures::clear_all()
{
    map_of_towns.clear();
    closest_dist = NO_DISTANCE;
    closest_town = NO_TOWNID;
    furthest_dist = NO_DISTANCE;
    furthest_town = NO_TOWNID;
    vector_all_roads.clear();
}

bool Datastructures::add_town(TownID id, const Name &name, Coord coord, int tax)
{
    if (map_of_towns.find(id) != map_of_towns.end())
    {
        return false;
    }
    Single_town newtown;
    newtown.name_ = name;
    newtown.coordinates_ = coord;
    newtown.tax_ = tax;

    Coord origin = {0,0};
    newtown.distance_ = distance_between_points(origin,coord);

    if (closest_town == NO_TOWNID or newtown.distance_ <= closest_dist)
    {
        closest_town = id;
        closest_dist = newtown.distance_;
    }
    if (furthest_town == NO_TOWNID or newtown.distance_ > furthest_dist)
    {
        furthest_town = id;
        furthest_dist = newtown.distance_;
    }
    map_of_towns.insert({id,newtown});
    return true;
}

Name Datastructures::get_town_name(TownID id)
{
    if (map_of_towns.find(id) == map_of_towns.end())
    {
        return NO_NAME;
    }
    return map_of_towns[id].name_;
}

Coord Datastructures::get_town_coordinates(TownID id)
{
    if (map_of_towns.find(id) == map_of_towns.end())
    {
        return NO_COORD;
    }
    return map_of_towns[id].coordinates_;
}

int Datastructures::get_town_tax(TownID id)
{
    if (map_of_towns.find(id) == map_of_towns.end())
    {
        return NO_VALUE;
    }
    return map_of_towns[id].tax_;
}

std::vector<TownID> Datastructures::all_towns()
{
    std::vector<TownID> vec;
    for (auto i : map_of_towns)
    {
        vec.push_back(i.first);
    }
    return vec;
}

std::vector<TownID> Datastructures::find_towns(const Name &name)
{
    std::vector<TownID> vec;
    for (auto i : map_of_towns)
    {
        if (i.second.name_ == name)
        {
            vec.push_back(i.first);
        }
    }
    return vec;
}

bool Datastructures::change_town_name(TownID id, const Name &newname)
{
    if (map_of_towns.find(id) == map_of_towns.end())
    {
        return false;
    }
    map_of_towns[id].name_ = newname;
    return true;
}

std::vector<TownID> Datastructures::towns_alphabetically()
{
    std::pair<Name,TownID> pair1;
    std::vector<std::pair<Name,TownID>> vec1;
    for (auto i : map_of_towns)
    {
        pair1 = std::make_pair(i.second.name_,i.first);
        vec1.push_back(pair1);
    }
    std::sort(vec1.begin(),vec1.end());
    std::vector<TownID> vec2;
    for (auto i : vec1)
    {
        vec2.push_back(i.second);
    }
    return vec2;
}

std::vector<TownID> Datastructures::towns_distance_increasing()
{
    std::pair<int,TownID> pair1;
    std::vector<std::pair<int,TownID>> vec1;

    for (auto i : map_of_towns)
    {
        pair1 = std::make_pair(i.second.distance_,i.first);
        vec1.push_back(pair1);
    }
    std::sort(vec1.begin(),vec1.end());
    std::vector<TownID> vec2;
    for (auto i : vec1)
    {
        vec2.push_back(i.second);
    }
    return vec2;
}

TownID Datastructures::min_distance()
{
    return closest_town;

}

TownID Datastructures::max_distance()
{
    return furthest_town;
}

bool Datastructures::add_vassalship(TownID vassalid, TownID masterid)
{
    if (map_of_towns.find(vassalid) == map_of_towns.end()
            or map_of_towns.find(masterid) == map_of_towns.end())
    {
        return false;
    }
    map_of_towns[vassalid].parent_town = masterid;
    map_of_towns[masterid].vector_of_vassals.push_back(vassalid);
    return true;
}

std::vector<TownID> Datastructures::get_town_vassals(TownID id)
{
    if (map_of_towns.find(id) == map_of_towns.end())
    {

        return {NO_TOWNID};
    }

    std::vector<TownID> vec;
    for (auto i : map_of_towns[id].vector_of_vassals)
    {
        vec.push_back(i);
    }
    return vec;
}

std::vector<TownID> Datastructures::taxer_path(TownID id)
{
    if (map_of_towns.find(id) == map_of_towns.end())
    {
        return {NO_TOWNID};
    }
    TownID currtown = id;
    std::vector<TownID> vec = {id};
    while (map_of_towns.at(currtown).parent_town !=  NO_TOWNID)
    {
        vec.push_back(map_of_towns[currtown].parent_town);
        currtown = map_of_towns.at(currtown).parent_town;
    }
    return vec;
}

bool Datastructures::remove_town(TownID id)
{
    if (map_of_towns.find(id) == map_of_towns.end())
    {
        return false;
    }

    // Jos poistettavasta kaupungista lähtee teitä, ne pitää poistaa
    std::vector<TownID> roads_from = get_roads_from(id);
    for(auto i : roads_from)
    {
        remove_road(i,id);
    }

    if (map_of_towns[id].parent_town != NO_TOWNID)
    {
        for (auto i : map_of_towns[id].vector_of_vassals)
        {
            add_vassalship(i,map_of_towns[id].parent_town);
        }
        TownID parent_town = map_of_towns[id].parent_town;

        auto iterator = std::find(map_of_towns[parent_town].vector_of_vassals.begin(),
                            map_of_towns[parent_town].vector_of_vassals.end(),
                            id);

        map_of_towns[parent_town].vector_of_vassals.erase(iterator);
    }

    map_of_towns.erase(id);
    if (id == closest_town or id == furthest_town)
    {
        update_distance();
    }

    return true;
}

std::vector<TownID> Datastructures::towns_nearest(Coord coord)
{
    std::pair<int,TownID> pair1;
    std::vector< std::pair<int,TownID>> vec;
    for (auto i : map_of_towns)
    {
        int dist = distance_between_points(coord,i.second.coordinates_);
        pair1 = std::make_pair(dist,i.first);
        vec.push_back(pair1);
    }
    std::sort(vec.begin(),vec.end());
    std::vector<TownID> vec2;
    for (auto i : vec)
    {
        vec2.push_back(i.second);
    }
    return vec2;
}

std::vector<TownID> Datastructures::longest_vassal_path(TownID id)
{
    if (map_of_towns.find(id) == map_of_towns.end())
    {
        return {NO_TOWNID};
    }
    curr_longest_depth = 0;
    curr_base_vassal = NO_TOWNID;
    longest_tax_path_recursive(id,1);

    std::vector<TownID> vec;
    TownID curr_town = curr_base_vassal;
    while (curr_town != id)
    {
        vec.push_back(curr_town);
        curr_town = map_of_towns[curr_town].parent_town;
    }
    vec.push_back(id);
    std::reverse(vec.begin(),vec.end());
    return vec;
}

void Datastructures::longest_tax_path_recursive(TownID id, int depth)
{
    if (map_of_towns[id] .vector_of_vassals.size() == 0)
    {
        if (depth > curr_longest_depth)
        {
            curr_longest_depth = depth;
            curr_base_vassal = id;
            return;
        }
    }
    depth++;
    for (auto i : map_of_towns[id].vector_of_vassals)
    {
        longest_tax_path_recursive(i,depth);
    }
}


int Datastructures::total_net_tax(TownID id)
{
    if (map_of_towns.find(id) == map_of_towns.end())
    {
        return NO_VALUE;
    }

    if (map_of_towns[id].parent_town == NO_TOWNID)
    {
        return vassal_net_tax_recursive(id);
    }

    else
    {
        int total_tax = vassal_net_tax_recursive(id);
        int tenth_of_tax = total_tax * 0.1;
        return (total_tax - tenth_of_tax);
    }
}

int Datastructures::vassal_net_tax_recursive(TownID id)
{
    if (map_of_towns[id] .vector_of_vassals.empty())
    {
        return map_of_towns[id].tax_;
    }

    int tax_from_vassals = 0;
    for (auto i : map_of_towns[id].vector_of_vassals)
    {
        tax_from_vassals += vassal_net_tax_recursive(i) * 0.1;
    }
    return (tax_from_vassals + map_of_towns[id].tax_);
}


int Datastructures::distance_between_points(Coord first, Coord second)
{
    int dist = sqrt((first.x-second.x)*(first.x-second.x) + (first.y-second.y)*(first.y-second.y));
    return dist;
}


void Datastructures::update_distance()
{
    if (map_of_towns.empty())
    {
        closest_town = NO_TOWNID;
        closest_dist = NO_DISTANCE;

        furthest_town = NO_TOWNID;
        furthest_dist = NO_DISTANCE;
        return;
    }

    std::pair<int,TownID> pair1;
    std::vector< std::pair<int,TownID>> vec;
    for (auto i : map_of_towns)
    {
        pair1 = std::make_pair(i.second.distance_,i.first);
        vec.push_back(pair1);
    }

    auto [min, max] = std::minmax_element(begin(vec), end(vec));

    closest_town = min->second;
    closest_dist = min->first;

    furthest_town = max->second;
    furthest_dist = max->first;
}

//
// Phase 2 operations
//

void Datastructures::clear_roads()
{
    vector_all_roads.clear();
    for(auto &i: map_of_towns)
    {
        i.second.vector_roads_to.clear();
    }
    reset_visits();
}

std::vector<std::pair<TownID, TownID>> Datastructures::all_roads()
{
    return vector_all_roads;
}

bool Datastructures::add_road(TownID town1, TownID town2)
{
    if (map_of_towns.find(town1) == map_of_towns.end() or
        map_of_towns.find(town2) == map_of_towns.end())
    {
        return false;
    }
    for (auto i : map_of_towns.at(town1).vector_roads_to)
    {
        if(i == town2)
        {
            return false;
        }
    }
    std::pair<TownID,TownID> new_road;
    if(town1 > town2)
    {
        new_road = {town2,town1};
    }
    else
    {
         new_road = {town1,town2};
    }
    vector_all_roads.push_back(new_road);
    map_of_towns[town1].vector_roads_to.push_back(town2);
    map_of_towns[town2].vector_roads_to.push_back(town1);
    return true;
}

std::vector<TownID> Datastructures::get_roads_from(TownID id)
{
   if (map_of_towns.find(id) == map_of_towns.end())
    {
        return {};
    }
    return map_of_towns.at(id).vector_roads_to;
}

std::vector<TownID> Datastructures::any_route(TownID fromid, TownID toid)
{
    return least_towns_route(fromid,toid);
}

bool Datastructures::remove_road(TownID town1, TownID town2)
{
    if (map_of_towns.find(town1) == map_of_towns.end() or
        map_of_towns.find(town2) == map_of_towns.end())
    {
        return false;
    }
    for (auto &i : map_of_towns.at(town1).vector_roads_to)
    {
        if(i == town2)
        {
            // Poistetaan tie town1 structista
            auto iterator = std::find(map_of_towns.at(town1).vector_roads_to.begin(),
                              map_of_towns.at(town1).vector_roads_to.end(),
                              town2);

            map_of_towns.at(town1).vector_roads_to.erase(iterator);

            // Poistetaan tie town2 structista
            iterator = std::find(map_of_towns.at(town2).vector_roads_to.begin(),
                              map_of_towns.at(town2).vector_roads_to.end(),
                              town1);
            map_of_towns.at(town2).vector_roads_to.erase(iterator);

            // Poistetaan tie vector_all_roads
            std::pair<TownID,TownID> pair;
            if(town1 > town2)
            {
                pair = std::make_pair(town2,town1);
            }
            else
            {
                pair = std::make_pair(town1,town2);
            }

            auto sec_iterator = std::find(vector_all_roads.begin(),
                                 vector_all_roads.end(),
                                 pair);
            vector_all_roads.erase(sec_iterator);
            return true;
        }
    }
    return false;

}

std::vector<TownID> Datastructures::least_towns_route(TownID fromid, TownID toid)
{
    if (map_of_towns.find(fromid) == map_of_towns.end() or
        map_of_towns.find(toid) == map_of_towns.end())
    {
        return {};
    }
    // Suoritetaan BFS väärinpäin, jotta ei tarvitse kääntää reittivektoria
    TownID reverse_fromID = toid;
    TownID reverse_toID = fromid;

    reset_visits();
    map_of_towns.at(reverse_fromID).visited = true;
    std::deque<TownID> TownID_deque;
    TownID_deque.push_back(reverse_fromID);
    TownID curr_Townid;

    while (!TownID_deque.empty())
    {
        curr_Townid = TownID_deque.front();
        TownID_deque.pop_front();
        for(auto neighbour : map_of_towns.at(curr_Townid).vector_roads_to)
        {
            if (map_of_towns.at(neighbour).visited == false)
            {
                map_of_towns.at(neighbour).visited = true;
                map_of_towns.at(neighbour).reached_from = curr_Townid;
                TownID_deque.push_back(neighbour);
            }

            if (neighbour == reverse_toID)
            {
                return get_route_vector(reverse_fromID,reverse_toID);
            }
        }
    }
    return {NO_TOWNID};
}

std::vector<TownID> Datastructures::road_cycle_route(TownID startid)
{
    if (map_of_towns.find(startid) == map_of_towns.end())
    {
        return {};
    }
    reset_visits();
    loop_vector.clear();
    start_town = startid;
    depth_first_recursive(startid,NO_TOWNID);
    return loop_vector;
}

void Datastructures::depth_first_recursive(TownID currentID, TownID previousID)
{
    map_of_towns.at(currentID).visited = true;
    //Käydään naapuri solmut läpi
    for (auto neighbour : map_of_towns.at(currentID).vector_roads_to)
    {
        // On jo löytynyt silmukka
        if(!loop_vector.empty())
        {
            return;
        }

        // Löytyi napurisolmu jossa ei olla käyty
        else if(!map_of_towns.at(neighbour).visited)
        {
            if(neighbour != previousID)
            {
                map_of_towns.at(neighbour).reached_from = currentID;
            }
            depth_first_recursive(neighbour,currentID);
        }

        // Jos naapurisolmu on solmu, josta äsken tultiin
        else if (neighbour == previousID)
        {
            continue;
        }

        // Löytyi  silmukka
        else
        {
            // Aloitetaan silmukka toisiksi viimeisestä kaupungista ja lisätään vektorin loppuun viimeinen kaupunki
            loop_vector = get_route_vector(start_town,currentID);
            std::reverse(loop_vector.begin(),loop_vector.end());
            loop_vector.push_back(neighbour);
            return;
        }
    }
}

std::vector<TownID> Datastructures::shortest_route(TownID fromid, TownID toid)
{
    if (map_of_towns.find(fromid) == map_of_towns.end() or
        map_of_towns.find(toid) == map_of_towns.end())
    {
        return {};
    }
    // Kutsutaan väärinpäin A* algoritmiä, jotta ei tarvitse kääntää saatua vektoria
    return A_star(toid,fromid);
}

std::vector<TownID> Datastructures::A_star(TownID startID, TownID goalID)
{
    reset_visits();
    std::priority_queue<std::pair<int,TownID>> prio_queue;
    map_of_towns.at(startID).cost = 0;
    map_of_towns.at(startID).estimated_cost = - distance_between_towns(startID,goalID);

    prio_queue.push(std::make_pair(map_of_towns.at(startID).estimated_cost,startID));
    std::pair<int,TownID> curr_town_pair;
    int curr_estimated_cost;

    // Priority queue on järjestetty estimated_costin mukaan.
    // estimated_cost on negatiivinen, jotta priority queue järjestää ne oikein
    while (!prio_queue.empty())
    {
        curr_town_pair = prio_queue.top();
        prio_queue.pop();

        // Päästiin haluttuun kaupunkiin
        if(curr_town_pair.second == goalID)
        {
            return get_route_vector(startID,goalID);
        }

        // Käydään läpi priority queuen alkion naapurit
        for (auto &neighbour : map_of_towns.at(curr_town_pair.second).vector_roads_to)
        {
            relax(neighbour,curr_town_pair.second,goalID);

            // Lisätään löydetyt naapurit queuehun jos niissä ei olla käyty
            if(map_of_towns.at(neighbour).visited == false)
            {
                // Distance_between_towns ja map_of_towns.at(neighbour).cost ovat positiivisia joten ne pitää vaihtaa negatiivisiksi
                curr_estimated_cost = -1 * (distance_between_towns(neighbour,goalID) + map_of_towns.at(neighbour).cost);
                prio_queue.push(std::make_pair(curr_estimated_cost,neighbour));
            }
        }
        map_of_towns.at(curr_town_pair.second).visited = true;
    }
    return {NO_TOWNID};
}

void Datastructures::relax(TownID neighbour, TownID curr_town, TownID goalID)
{
    // new_cost on positiivinen
    int new_cost = distance_between_towns(neighbour,curr_town);

    // .cost on positiivinen, katsotaan onko aiempi reitti pidempi kuin löydetty reitti
    if(map_of_towns.at(neighbour).cost > new_cost + map_of_towns.at(curr_town).cost)
    {
        // .cost on positiivinen ja .estimated_cost on negatiivinen johtuen priority_queuen järjestyksestä
        map_of_towns.at(neighbour).cost = new_cost + map_of_towns.at(curr_town).cost;
        map_of_towns.at(neighbour).estimated_cost = -1 * (distance_between_towns(neighbour,goalID) + map_of_towns.at(neighbour).cost);
        map_of_towns.at(neighbour).reached_from = curr_town;
    }
}

Distance Datastructures::trim_road_network()
{
    // Kruskalin algoritmi
    std::vector<std::pair<int,std::pair<TownID,TownID>>> temp_roads;
    std::pair<TownID,TownID> curr_town;
    int road_length;

    for(auto i : vector_all_roads)
    {
        road_length = distance_between_towns(i.first,i.second);
        temp_roads.push_back(std::make_pair(road_length,i));
    }

    clear_roads();
    std::sort(temp_roads.begin(),temp_roads.end());
    int total_road_length = 0;
    // Lisätään teitä yksitellen takaisin alkaen lyhimmästä
    for(auto k : temp_roads)
    {
        // Katsotaan löytyykö jo olemassa olevaa reittiä kaupunkien välillä
        TownID temp = least_towns_route(k.second.first,k.second.second).at(0);
        if (temp == NO_TOWNID)
        {
            total_road_length += k.first;
            add_road(k.second.first,k.second.second);
        }
    }
    return total_road_length;
}

int Datastructures::distance_between_towns(TownID town1, TownID town2)
{
    Coord first = map_of_towns.at(town1).coordinates_;
    Coord second = map_of_towns.at(town2).coordinates_;
    int dist = sqrt((first.x-second.x)*(first.x-second.x) + (first.y-second.y)*(first.y-second.y));
    return dist;
}

void Datastructures::reset_visits()
{
    for(auto &i : map_of_towns)
    {
        i.second.visited = false;
        i.second.reached_from = NO_TOWNID;
        i.second.cost = -1 * negative_infinity;
        i.second.estimated_cost = negative_infinity;
    }
}

std::vector<TownID> Datastructures::get_route_vector(TownID startID, TownID goalID)
{
    std::vector<TownID> return_vec;
    TownID iterator = goalID;
    // Lisätään vektoriin kaupungit mistä on tultu
    while(iterator != startID)
    {
        return_vec.push_back(iterator);
        iterator = map_of_towns.at(iterator).reached_from;
    }
    return_vec.push_back(startID);
    return return_vec;
}
