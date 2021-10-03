// MIT License

// Copyright (c) 2021 Radhakrishnan Thangavel

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <iostream>
#include <unordered_map>
#include <unordered_set>
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
#include <map>
#include <sstream>

// FlashLogger
#include "FLogManager.h"
#include <config.h>

// RkUtil [START]
//void* operator new(size_t size)
//{
//    static std::size_t c;
//    FLOG_INFO<< "Alocating: " << ++c << " times";
//    void * p = ::operator new(size);
//    return p;
//}
namespace RkUtil{

std::vector<std::string> Split(const std::string& input, const char delimiter){

    std::vector<std::string> result;
    std::stringstream ss(input);
    std::string s;
    while (std::getline(ss, s, delimiter)) {
        result.push_back(s);
    }

    return result;
}

#define GETTER(OBJ) public:\
    const decltype(OBJ)& get##OBJ() const { return OBJ; }

// With this Graph implementation will never need to be changed for stored data ("types") Closed for Modification Open for extension
template<typename T>
struct GraphObject{
    T data;
};

template<>
struct GraphObject<std::string>{
    explicit GraphObject(std::string&& d) noexcept{
        d = std::exchange(data, d);
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
using Vertex_WPtr = std::weak_ptr<Vertex<T>>;
template<typename T>
using Edge_Ptr = std::shared_ptr<Edge<T>>;
template<typename T>
class Graph;
template<typename T>
using Graph_Ptr = std::unique_ptr<Graph<T>>;

template<typename T>
class Graph{

public:
    // node is copy constructed here so move this memory to Vertex ownership. it dosent matter
    // caller of this function did move or copy. from here vertex take care of this memory.
    void AddNode(T node, std::vector<T> neighbours){
        for (auto& i : neighbours){
            addEdge(node, i);
        }
    }

    void addEdge(T& n1, T& n2){
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
    // Read intensive than Write so used array instead of linked list
    std::vector<Edge_Ptr<T>> mEdges;
    std::vector<Vertex_Ptr<T>> mVertexes;

    GETTER(mEdges);
    GETTER(mVertexes);
};

template<typename T>
class Vertex {

    using VertecesList = std::unordered_set<Vertex_Ptr<T>>;

public:
    Vertex(T&& data):mData(std::move(data)) {    }

    static Vertex_Ptr<T> GetNewVertex(T& data) noexcept{
        // make_shared have strong exception safety.
        return std::make_shared<Vertex>(std::move(data));
    }

    // Vertex is always identified by its data
    friend bool operator==(const Vertex_Ptr<T>& lhs, const T& rhs){
        return lhs->mData.data == rhs;
    }

    void AddNeighbour(const Edge_Ptr<T>& e, const Vertex_Ptr<T>& v){
        mEdges.push_back(e);
        mNeighbours.insert(v);
    }

    int getDegree() const{
        return mEdges.size();
    }

    // let not return "const" as the copy ellision will be supressed. let compiler do good.
    T Data() const {
        return mData.data;
    }

    friend std::ostream& operator<<(std::ostream& os, const Vertex<T>& v){
        // os is inconsistent accross compiler on order of execution so avoided
        os << "[ " << v.mData.data << " ]";
        return os;
    }

private:
    GraphObject<T> mData;
    std::vector<Edge_Ptr<T>> mEdges;
    VertecesList mNeighbours;

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
    Vertex_WPtr<T> mV1;
    Vertex_WPtr<T> mV2;

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
    virtual ~Command() { }
};

template<typename T = std::string>
class IterativeSolver : public Command<T>{

public:
    ~IterativeSolver() override{
        mBuildOrder.clear();
    }

    bool API_1() override {
        return mIsCyclic;
    }

    std::vector<T> API_2() override {
        std::vector<T> ret;
        ret.reserve(mBuildOrder.size());
        for (auto itr = mBuildOrder.begin(); itr != mBuildOrder.end(); itr++){
            ret.emplace_back((*itr)->Data());
        }
        return ret;
    }

    void execute(const RkUtil::Graph_Ptr<T>& graph) override {
        FLOG_INFO << "IterativeSolver in action!!";
        std::tie(mIsCyclic, mBuildOrder) = isCyclic(graph);
    }

private:
    /*
     * Kahn’s algorithm for Topological Sorting
     * A DAG has at least one vertex with in-degree 0 and one vertex with out-degree 0.
    */
    std::tuple<bool, std::vector<RkUtil::Vertex_Ptr<T>>>
    isCyclic(const RkUtil::Graph_Ptr<T>& graph){

            std::unordered_map<RkUtil::Vertex_Ptr<T>, std::vector<RkUtil::Vertex_Ptr<T>>> depList;
            std::unordered_map<RkUtil::Vertex_Ptr<T>, int> inDegree;
            static std::vector<RkUtil::Vertex_Ptr<T>> Emptyorder;
            std::queue<RkUtil::Vertex_Ptr<T>> q;
            const auto& cachemVertexes = graph->getmVertexes();

            // O(V #no. of vertices# + E #no. of edges#)
            {
                for (const auto& e : graph->getmEdges()) {
                    depList[e->getmV2().lock()].push_back(e->getmV1().lock());
                }

                std::for_each(cachemVertexes.begin(), cachemVertexes.end(), [&q, &inDegree](const auto& v){
                    const int d = v->getDegree();
                    if ( d == 0) {
                        q.push(v);
                    }
                    inDegree[v] = d;
                });
            }

            while (!q.empty()) {
              const auto node = q.front();
              q.pop();
              mBuildOrder.push_back(node);

              auto itr = depList.find(node);
              if (itr != depList.end()) {
                for (const auto& neighbor : itr->second) {
                  inDegree[neighbor]--;

                  if (inDegree[neighbor] == 0) {
                    q.push(neighbor);
                  }
                }
              }
            }

            if (mBuildOrder.size() == cachemVertexes.size()) {
              return std::make_tuple(false, mBuildOrder);
            }

            return std::make_tuple(true, Emptyorder);
    }

private:
    bool mIsCyclic;
    std::vector<RkUtil::Vertex_Ptr<T>> mBuildOrder;
};

template<typename T = std::string>
class RecursiveSolver : public Command<T>{

    struct Den{
    public:
        enum class color{
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
    ~RecursiveSolver() override{
        mBuildOrder.clear();
    }

    bool API_1() override {
        return mIsCyclic;
    }

    std::vector<T> API_2() override {
        std::vector<T> ret;
        ret.reserve(mBuildOrder.size());
        for (auto itr = mBuildOrder.begin(); itr != mBuildOrder.end(); itr++){
            ret.emplace_back((*itr)->Data());
        }
        return ret;
    }

    void execute(const RkUtil::Graph_Ptr<T>& graph) override{
        FLOG_INFO << "RecursiveSolver in action!!";
        std::tie(mIsCyclic, mBuildOrder) = isCyclic(graph);
    }

    std::tuple<bool, std::vector<RkUtil::Vertex_Ptr<T>>>
                isCyclic(const RkUtil::Graph_Ptr<T>& graph){

        Den d;
        static std::vector<RkUtil::Vertex_Ptr<T>> Emptyorder;

        // Let "Den" know all the vertices
        for (const auto& vertex : graph->getmVertexes()) {
            d.makeColored(vertex, Den::color::WHITE);
        }

        // O(V #no. of vertices# + E #no. of edges#)
        {
            for (const auto& v : graph->getmVertexes()) {
                if (!d.isVisited(v)){
                    if(dfs(v, d)) {
                        return std::make_tuple(true, Emptyorder);
                    }
                }
            }
        }

        return std::make_pair(false, mBuildOrder);
    }

    bool dfs(const RkUtil::Vertex_Ptr<T>& current, Den& d) {
        d.makeColored(current, Den::color::GRAY);
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
        d.makeColored(current, Den::color::BLACK);
        mBuildOrder.push_back(current);
        return false;
    }

private:
    bool mIsCyclic;
    std::vector<RkUtil::Vertex_Ptr<T>> mBuildOrder;
};

using namespace std::literals;

/*
 * Desiged per SOLID principles and Strong exception safety
 * Space Complexity O(V) as Edges will only hold a shallow copy adjoining verteces.
 * NOTE for further extension:
 * If file is too large for main memory exploit virtual memory or use memory mapping eg: boost library have filesystem mapping APIs which can map at
 * different offsets[start, end] so any file can be processssed with that approach
*/
class InputParser
{
public:
    static RkUtil::Graph_Ptr<std::string> GenerateGraphFromFile(const std::string_view filename, bool recursive_method = true){

        RkUtil::Graph_Ptr<std::string> graph;
        std::ifstream input(filename.data());
        if (!input){
            std::cerr << "inpuut file not found";
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
                        std::vector<std::string> v = RkUtil::Split(line, ' ');
                        if (noOfEntries == -1)
                            throw c;
                        if (!graph)
                            graph = std::make_unique<RkUtil::Graph<std::string>>();

                        auto neighboursstart = std::next(std::begin(v));
                        if (recursive_method){
                            std::vector<std::string> neighbours;
                            neighbours.resize(v.size() - 1);
                            std::swap_ranges(v.begin()+1, v.end(), neighbours.begin());
                            graph->AddNode(std::move(v[0]), std::move(neighbours));
                            for (const auto& i : v)
                                std::cout << i << std::endl;
                        }/*else{
                            for (auto itr = neighboursstart; itr != v.end(); itr++){
                                graph->addEdge(*itr, v[0]);
                            }
                        }*/
                    }
                }
            }
            auto end = std::chrono::steady_clock::now();
            //FLOG_INFO << "Time to complete graph construction: " << (end - start)/1s << "s.";
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

LEVEL FLogManager::mCurrentLevel = LEVEL::CRIT;
GRANULARITY FLogManager::mCurrentGranularity = GRANULARITY::FULL;

int main(int argc, char *argv[])
{    
    std::unique_ptr<FLogConfig> config = std::make_unique<FLogConfig>([](flashlogger_config_data &d, boost::program_options::options_description &desc){
            desc.add_options()
                    ("FlashLogger.size_of_ring_buffer", boost::program_options::value<short>(&d.size_of_ring_buffer)->default_value(50), "size of buffer to log")
                    ("FlashLogger.log_file_path", boost::program_options::value<std::string>(&d.log_file_path)->default_value("../"), "log file path")
                    ("FlashLogger.log_file_name", boost::program_options::value<std::string>(&d.log_file_name)->default_value("flashlog.txt"), "log file name")
                    ("FlashLogger.run_test", boost::program_options::value<short>(&d.run_test)->default_value(1), "choose to run test")
                    ("FlashLogger.server_ip", boost::program_options::value<std::string>(&d.server_ip)->default_value("localhost"), "microservice server IP")
                    ("FlashLogger.server_port", boost::program_options::value<std::string>(&d.server_port)->default_value("50051"), "microservice server port");
        });

    try {

        config->parse(argc, argv);
    }
    catch(std::exception const& e) {

        std::cout << e.what();
        return 0;
    }

    FLogManager::globalInstance(std::move(config)).SetCopyrightAndStartService(s_copyright);

    FLogManager::globalInstance().SetLogGranularity("BASIC");
    FLogManager::globalInstance().SetLogLevel("INFO");

    if(argc < 1){
        std::cerr << "Usage: ./solution <absolutepath/filename.txt> ";
        return 0;
    }

    const std::string_view input_file(argv[1]);
    if (const RkUtil::Graph_Ptr<std::string> graph = InputParser::GenerateGraphFromFile(input_file);
        graph && graph->isValid()){

        const int MAX_HW_STACK_DEPTH = 500;
        std::size_t max_depth_of_graph = graph->getmVertexes().size() + graph->getmEdges().size();
        std::unique_ptr<Command<std::string>> c;
        if (max_depth_of_graph > MAX_HW_STACK_DEPTH){
            c.reset(new IterativeSolver<std::string>());
        }else{
            c.reset(new RecursiveSolver<std::string>());
        }
        c->execute(graph);
        {
            // API 1
            FLOG_INFO << "Was there any cyclic dependency ? : " << c->API_1();
            // API 2
            FLOG_INFO << "A build order for the products : ";
            const auto& v = c->API_2();
            for (const auto& i : v){
                FLOG_INFO << i.data() << " ";
            }
            std::copy(v.begin(), v.end(), std::ostream_iterator<std::string>(std::cout, " "));
        }
    }

    return 0;
}
