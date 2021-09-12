#include <iostream>
#include <unordered_map>
#include <vector>
#include <memory>
#include <iostream>
#include <vector>
#include <type_traits>
#include <regex>
#include <fstream>
#include <chrono>
#include <iterator>
#include <algorithm>
#include <queue>

// RkUtil [START]
//void* operator new(size_t size)
//{
//    static std::size_t c;
//    std::cout<< "Alocating: " << ++c << " times" << std::endl;
//    void * p = ::operator new(size);
//    return p;
//}
namespace RkUtil{

std::vector<std::string> Split(const std::string& input, const std::string& delimiter){

    std::vector<std::string> ret;
    std::stringstream ss;
    ss << "[^\\" << delimiter << "]+";
    std::regex r(ss.str());

    for (std::sregex_iterator i = std::sregex_iterator(input.begin(), input.end(), r);
         i != std::sregex_iterator(); ++i){
        std::smatch m = *i;
        ret.push_back(m.str());
    }

    return ret;
}

#define GETTER(OBJ) public:\
    const decltype(OBJ)& get##OBJ() const { return OBJ; }

template<typename T>
struct GraphObject{
    T data;
};

template<>
struct GraphObject<std::string>{
    GraphObject(const std::string& d) noexcept
        :data(d){ }

    friend bool operator<(const std::string& lhs, const std::string& rhs){
        return lhs < rhs;
    }
    bool operator==(const GraphObject<std::string>& rhs) const{
        return (this->data == rhs.data);
    }
    std::string data;
};

template<typename T>
class Vertex;
template<typename T>
class Edge;
template<typename T>
using Vertex_Ptr = std::shared_ptr<Vertex<T>>;
template<typename T>
using Edge_Ptr = std::shared_ptr<Edge<T>>;
template<typename T>
class Graph;
template<typename T>
using Graph_Ptr = std::unique_ptr<Graph<T>>;

template<typename T>
class Graph{

public:
    void AddNode(const T&& node, const std::vector<T>&& neighbours){
        for (auto& i : neighbours){
            addEdge(node, i);
        }
    }

    void addEdge(const T& n1, const T& n2){
        auto find_node = [this](const auto& node){
            return std::find(mVertexes.begin(), mVertexes.end(), node);
        };
        Vertex_Ptr<T> v1;
        Vertex_Ptr<T> v2;

        auto itrv1 = find_node(n1);
        if(itrv1 == mVertexes.end()){
            v1 = Vertex<T>::GetNewVertex(n1);
            mVertexes.push_back(v1);
        }else{
            v1 = *itrv1;
        }
        auto itrv2 = find_node(n2);
        if(itrv2 == mVertexes.end()){
            v2 = Vertex<T>::GetNewVertex(n2);
            mVertexes.push_back(v2);
        }else{
            v2 = *itrv2;
        }

        const Edge_Ptr<T>& newEdge = Edge<T>::GetNewEdge(v1, v2);
        mEdges.push_back(newEdge);
        v1->AddNeighbour(newEdge, v2);
    }

    bool isValid() { return !mVertexes.empty(); }
private:
    std::vector<Edge_Ptr<T>> mEdges;
    std::vector<Vertex_Ptr<T>> mVertexes;

    GETTER(mEdges);
    GETTER(mVertexes);
};

template<typename T>
class Vertex {

public:
    Vertex(const T& data):mData(data) {}

    static Vertex_Ptr<T> GetNewVertex(const T& data){
        return std::make_shared<Vertex>(data);
    }

    friend bool operator==(const Vertex_Ptr<T>& lhs, const T& rhs){
        return lhs->mData.data == rhs;
    }

    void AddNeighbour(const Edge_Ptr<T>& e, const Vertex_Ptr<T>& v){
        mEdges.push_back(e);
        mNeighbours.push_back(v);
    }

    int getDegree(){
        return mEdges.size();
    }

    T& Data() {
        return mData.data;
    }

    friend std::ostream& operator<<(std::ostream& os, const Vertex<T>& v){
        os << "[ " << v.mData.data << " ]";
        return os;
    }

private:
    GraphObject<T> mData;
    std::vector<Edge_Ptr<T>> mEdges;
    std::vector<Vertex_Ptr<T>> mNeighbours;

public:
    GETTER(mEdges);
    GETTER(mNeighbours);
};

template<typename T>
class Edge{

public:

    Edge(const Vertex_Ptr<T>& v1, const Vertex_Ptr<T>& v2)
        : mV1(v1), mV2(v2) { }

    static Edge_Ptr<T> GetNewEdge(const Vertex_Ptr<T>& v1, const Vertex_Ptr<T>& v2){
       return std::make_shared<Edge>(v1, v2);
    }

private:
    Vertex_Ptr<T> mV1;
    Vertex_Ptr<T> mV2;

public:
    GETTER(mV1);
    GETTER(mV2);
};

}// RkUtil [END]

template<typename T= std::string>
class Command{
public:
    virtual void execute(const RkUtil::Graph_Ptr<T>& graph) = 0;
    virtual bool API_1() = 0;
    virtual std::vector<T> API_2() = 0;
};

template<typename T= std::string>
class IterativeSolver : public Command<T>{
public:
    bool API_1() override{
        return mIsCyclic;
    }

    std::vector<T> API_2() override{
        std::vector<T> ret;
        ret.reserve(mBuildOrder.size());
        for (auto itr = mBuildOrder.rbegin(); itr != mBuildOrder.rend(); itr++){
            ret.push_back((*itr)->Data());
        }
        return ret;
    }

public:

    void execute(const RkUtil::Graph_Ptr<T>& graph) override {
        std::cout << "IterativeSolver" << " in action!!" << std::endl;
        std::tie(mIsCyclic, mBuildOrder) = isCyclic(graph);
    }

    std::tuple<bool, std::vector<RkUtil::Vertex_Ptr<T>>>
    isCyclic(const RkUtil::Graph_Ptr<T>& graph){

            bool isPossible = true;
            std::unordered_map<RkUtil::Vertex_Ptr<T>, std::vector<RkUtil::Vertex_Ptr<T>>> adjList;
            std::map<RkUtil::Vertex_Ptr<T>, int> indegree;
            std::vector<RkUtil::Vertex_Ptr<T>> topologicalOrder;
            static std::vector<RkUtil::Vertex_Ptr<T>> order;

            for (const auto& e : graph->getmEdges()) {
                adjList[e->getmV2()].push_back(e->getmV1());
                indegree[e->getmV1()] += 1;
            }

            std::queue<RkUtil::Vertex_Ptr<T>> q;
            for (const auto& v : graph->getmVertexes()) {
              if (indegree[v] == 0) {
                q.push(v);
              }
            }

            int i = 0;
            // Process until the Q becomes empty
            while (!q.empty()) {
              const auto node = q.front();
              q.pop();
              mBuildOrder.push_back(node);

              auto itr = adjList.find(node);
              if (itr != adjList.end()) {
                for (const auto& neighbor : itr->second) {
                  indegree[neighbor]--;

                  if (indegree[neighbor] == 0) {
                    q.push(neighbor);
                  }
                }
              }
            }

            if (mBuildOrder.size() == graph->getmVertexes().size()) {
              return std::make_tuple(true, mBuildOrder);
            }

            return std::make_tuple(true, order);
    }

private:
    bool mIsCyclic;
    std::vector<RkUtil::Vertex_Ptr<T>> mBuildOrder;
};

template<typename T = std::string>
class RecursiveSolver : public Command<T>{

    struct Den{
    public:
        enum /*class */color{
            WHITE,
            GRAY,
            BLACK
        };
        inline void makeColored(const RkUtil::Vertex_Ptr<T>& node, color c){
            data[node] = c;
        }
        inline bool isVisited(const RkUtil::Vertex_Ptr<T>& n) {
            return (data[n] != color::WHITE);
        }
        inline bool isAlreadyExplored(const RkUtil::Vertex_Ptr<T>& n) {
            return (data[n] == color::BLACK);
        }
        inline bool isCurrentlyBeingExplored(const RkUtil::Vertex_Ptr<T>& n) {
            return (data[n] == color::GRAY);
        }
        std::unordered_map<RkUtil::Vertex_Ptr<T>, color> data;
    };

public:
    bool API_1() {
        return mIsCyclic;
    }

    std::vector<T> API_2() {
        std::vector<T> ret;
        ret.reserve(mBuildOrder.size());
        for (auto itr = mBuildOrder.rbegin(); itr != mBuildOrder.rend(); itr++){
            ret.push_back((*itr)->Data());
        }
        return ret;
    }

public:

    void execute(const RkUtil::Graph_Ptr<T>& graph) {
        std::cout << "RecursiveSolver" << " in action!!" << std::endl;
        std::tie(mIsCyclic, mBuildOrder) = isCyclic(graph);
    }

    std::tuple<bool, std::vector<RkUtil::Vertex_Ptr<T>>>
                isCyclic(const RkUtil::Graph_Ptr<T>& graph){

        Den d;
        static std::vector<RkUtil::Vertex_Ptr<T>> order;

        for (const auto& vertex : graph->getmVertexes()) {
            d.makeColored(vertex, Den::WHITE);
        }

        for (const auto& v : graph->getmVertexes()) {
            if (!d.isVisited(v)){
                if(dfs(v, d)) {
                    return std::make_tuple(true, order);
                }
            }
        }

        return std::make_pair(false, mBuildOrder);
    }

    bool dfs(const RkUtil::Vertex_Ptr<T>& current, Den& d) {
        d.makeColored(current, Den::GRAY);
        for(const auto& neighbour : current->getmNeighbours()) {
            if (!d.isAlreadyExplored(neighbour)) {
                if (d.isCurrentlyBeingExplored(neighbour)) {
                    return true;
                }
                if(dfs(neighbour, d)) {
                    return true;
                }
            }
        }
        d.makeColored(current, Den::BLACK);
        mBuildOrder.push_back(current);
        return false;
    }

private:
    bool mIsCyclic;
    std::vector<RkUtil::Vertex_Ptr<T>> mBuildOrder;
};

using namespace std::literals;

class InputParser
{
public:
    // Strong exception safety
    static RkUtil::Graph_Ptr<std::string> GenerateGraphFromFile(const std::string_view filename, bool recursive_method = true){

        RkUtil::Graph_Ptr<std::string> graph;
        std::ifstream input(filename.data());
        if (!input){
            std::cerr << "inpuut file not found" << std::endl;
            return graph;
        }

        try{
            auto start = std::chrono::steady_clock::now();
            std::size_t c = 0, noOfEntries = -1;
            for (std::string line; std::getline(input, line, '\n'); ) {
                if (!line.empty()) {
                    if (++c == 1){
                        noOfEntries = std::atoi(line.data());
                    }else{
                        const std::vector<std::string>& v = RkUtil::Split(line, " ");
                        if (noOfEntries == -1)
                            throw c;
                        if (!graph)
                            graph = std::make_unique<RkUtil::Graph<std::string>>();

                        auto neighboursstart = std::next(std::begin(v));
                        if (recursive_method){
                            std::vector<std::string> neighbours{neighboursstart, std::end(v)};
                            graph->AddNode(std::move(v[0]), std::move(neighbours));
                        }else{
                            for (auto itr = neighboursstart; itr != v.end(); itr++){
                                graph->addEdge(*itr, v[0]);
                            }
                        }
                    }
                }
            }
            auto end = std::chrono::steady_clock::now();
            //std::cout << "Time to complete graph construction: " << (end - start)/1s << "s." << std::endl;
        }catch(int c){
            std::cerr << "Input failed at line: " << c;
            input.close();
        }catch (...) {
            input.close();
        }

        input.close();
        return graph;
    }
};

int main(int argc, char *argv[])
{    
    if(argc < 1){
        std::cerr << "Usage: ./solution <absolutepath/filename.txt> " << std::endl;
        return 0;
    }
    const std::string_view input_file(argv[1]);
    if (const RkUtil::Graph_Ptr<std::string> graph = InputParser::GenerateGraphFromFile(input_file, false);
        graph->isValid()){

        int t = 0;
        while(++t < 3){

            std::unique_ptr<Command<std::string>> c;
            if (t == 1){
                c.reset(new RecursiveSolver<std::string>());
            }else{
                c.reset(new IterativeSolver<std::string>());
            }
            c->execute(graph);
            {
                // API 1
                std::cout << "Was there any cyclic dependency? : " << std::boolalpha << c->API_1() << std::endl;
                // API 2
                std::cout << "A build order for the products : ";
                const auto& v = c->API_2();
                std::copy(v.begin(), v.end(), std::ostream_iterator<std::string>(std::cout, " "));
                std::cout << std::endl;
            }
        }
    }

    std::cerr << "Something wrong!!" << std::endl;

    return 0;
}
