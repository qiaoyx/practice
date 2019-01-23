#ifndef TSP_H
#define TSP_H

#include <boost/graph/properties.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/metric_tsp_approx.hpp>
#include <boost/graph/biconnected_components.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/max_cardinality_matching.hpp>
#include <boost/graph/johnson_all_pairs_shortest.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>

#include <boost/math/cstdfloat/cstdfloat_types.hpp>

using namespace boost;
using namespace boost::graph;

#define LC_THIS (*this)

class comm_edge_t
{
public:
    comm_edge_t(const std::string &s, const std::string &t,
                const boost::float64_t &d)
        : source_(s), target_(t), d_(d), task_(false) {}
    virtual ~comm_edge_t() {}

    comm_edge_t operator=(const comm_edge_t &e) {
        source_ = e.source_;
        target_ = e.target_;
        d_ = e.d_;
        task_ = e.task_;
        return LC_THIS;
    }

    bool operator== (const comm_edge_t &e) {
        return (source_ == e.source_ && target_ == e.target_);
    }

    bool task() const { return task_; }
    void task(const bool &b) { task_ = b; }

    void reverse() {
        std::string tmp = source_;
        source_ = target_;
        target_ = tmp;
    }

public:
    std::string source_;        // 源顶点
    std::string target_;        // 目标顶点
    boost::float64_t d_;        // 路径长度
    bool task_;                 // 标识是否在本次执行的任务中
};

class robot_path
{
public:
    enum { k_vmax = 3000 /* 支持的最大顶点数 */ };
    struct edge_component_t {
        enum { num = 555 };
        typedef edge_property_tag kind;
    } edge_component;           // 强联通分支
    struct edge_color_t {
        enum { num = 556 };
        typedef edge_property_tag kind;
    } edge_color;               // 颜色属性，目前用于记录是否参与任务
    struct edge_art_t {
        enum { num = 557 };
        typedef edge_property_tag kind;
    } edge_art;                 // 割边标记，表示未增加倍边时是否为割边
    typedef struct edge_gradient_t {    // 涉及到角度间的运算：车头朝向、临接的两条边夹角
        enum { num = 558 };
        typedef edge_property_tag kind;
    } edge_gradient;            // 该边的斜率，垂直时无意义，怎么表示呢？

    struct vertex_pos_t {
        boost::float32_t x;
        boost::float32_t y;
        boost::float32_t z;
    };

    typedef property<vertex_name_t, std::string> vertex_pri_t;
    typedef property<edge_weight_t, boost::float64_t,
                     property<edge_component_t, std::size_t,
                              property<edge_color_t, bool,
                                       property<edge_art_t, bool> > > > edge_pri_t;
    typedef boost::adjacency_list<
        boost::vecS, boost::vecS, boost::undirectedS,
        vertex_pri_t, edge_pri_t> graph_t;
#define NULL_V graph_traits<graph_t>::null_vertex()
    typedef graph_traits<graph_t>::edge_iterator e_it_t;
    typedef graph_traits<graph_t>::vertex_iterator v_it_t;
    typedef graph_traits<graph_t>::edge_descriptor e_desc_t;
    typedef graph_traits<graph_t>::vertex_descriptor v_desc_t;
    typedef property_map<graph_t, edge_color_t>::type e_color_t;
    typedef property_map<graph_t, edge_component_t>::type e_comp_t;

    typedef std::vector<std::vector<boost::float64_t> > distance_t;

    typedef std::pair<e_desc_t, bool> tri_edge_t;
    typedef std::pair<v_desc_t, v_desc_t> vv_edge_t;
    typedef std::pair<std::string, std::string> str_edge_t;

public:
    robot_path() {}
    virtual ~robot_path() { clear(); }

    v_desc_t
    operator[] (const std::string &n) const { return index(n); }
    std::string
    operator[] (const v_desc_t &i) const { return name(i); }
    boost::float64_t
    operator[] (const e_desc_t &e) const { return get(edge_weight, graph_, e); }

    tri_edge_t
    operator[] (const vv_edge_t &vv) {
        return edge(vv.first, vv.second, graph_);
    }
    tri_edge_t
    operator[] (const str_edge_t &vv) {
        return LC_THIS[vv_edge_t(index(vv.first), index(vv.second))];
    }
    tri_edge_t
    operator[] (const comm_edge_t &vv) {
        return LC_THIS[str_edge_t(vv.source_, vv.target_)];
    }

    bool find(const std::string &) const;

    robot_path
    operator>> (const std::string &v) {
        if (!find(v))
            add_vertex(v, graph_);
        return LC_THIS;
    }
    robot_path
    operator>> (const comm_edge_t &e) {
        LC_THIS >> e.source_ >> e.target_;
        add_edge(LC_THIS[e.source_], LC_THIS[e.target_], e.d_, graph_);
        return LC_THIS;
    }

    v_desc_t ins_vertex(const std::string &v);
    tri_edge_t ins_edge(const comm_edge_t &e);

    robot_path
    operator- (e_desc_t &e) {
        remove_edge(e, graph_);
        return LC_THIS;
    }
    robot_path
    operator- (const comm_edge_t &e) {
        str_edge_t vv(e.source_, e.target_);
        tri_edge_t be = LC_THIS[vv];
        if (be.second)
            remove_edge(be.first, graph_);
        return LC_THIS;
    }
    robot_path
    operator- (v_desc_t &v) {
        clear_vertex(v, graph_);
        remove_vertex(v, graph_);
        return LC_THIS;
    }

    boost::uint32_t
    v_num() const { return num_vertices(graph_); }
    boost::uint32_t
    e_num() const { return num_edges(graph_); }
    boost::float64_t
    compare(const e_desc_t &u, const e_desc_t &v) const {
        return LC_THIS[u] - LC_THIS[v];
    }
    bool less(const e_desc_t &u, const e_desc_t &v) const {
        return LC_THIS[u] < LC_THIS[v];
    }
    bool great(const e_desc_t &u, const e_desc_t &v) const {
        return LC_THIS[u] > LC_THIS[v];
    }

    bool is_articulation(const v_desc_t &);
    bool is_articulation(const e_desc_t &);

    boost::float64_t
    shortest_path(const v_desc_t &, const v_desc_t &, std::list<v_desc_t> &);
    std::list<e_desc_t>
    shortest_path(const e_desc_t &, const e_desc_t &);
    std::vector<comm_edge_t>
    optimal_path(const std::string &, const std::string &,
                 std::vector<comm_edge_t> &);

    v_desc_t nearest(const v_desc_t&, const std::vector<v_desc_t>&);

private:
    void clear();
    void clear_distance();
    void assign_distance(const size_t &);

    std::string name(const v_desc_t &) const;
    v_desc_t index(const std::string &) const;

    size_t degree(const v_desc_t &v) const {
        return out_degree(v, graph_);
    }

    bool full_shortest_distance();
    robot_path extence(const std::vector<v_desc_t> &);
    std::list<e_desc_t>  min_conn_closer();
    std::list<e_desc_t>  min_conn_closer_ext();
    void matching(std::list<str_edge_t> &);
    void euler();
    std::list<std::string> fluery(const v_desc_t &, const v_desc_t &);

    void of_graph();

private:
    graph_t graph_;
    distance_t distance_;
    std::map<e_desc_t, size_t> count;
};

#if (defined(_WIN32) || defined(_WIN64))
struct out_edge_t {
    unsigned int in_;
    unsigned int out_;
    double d_;
    unsigned int t_;
};
extern "C" __declspec(dllexport) size_t __stdcall
calc_optimal_path(unsigned int, size_t, out_edge_t*);
#endif /* WIN */

#endif /* TSP_H */