#include <fstream>
#include <iostream>
#include <boost/bind.hpp>
// #include <boost/foreach.hpp>
// #include <boost/lambda/if.hpp>
// #include <boost/lambda/lambda.hpp>

#include "tsp.hpp"

// #define foreach boost::BOOST_FOREACH

void robot_path::clear()
{
    count.clear();
    clear_distance();
    graph_.clear();
}

void robot_path::clear_distance()
{
    for (size_t i = 0; i < distance_.size(); ++i)
        distance_[i].clear();
    distance_.clear();
}

void robot_path::assign_distance(const size_t &n)
{
    clear_distance();
    for (size_t i = 0; i < n; ++i)
        distance_.push_back(std::vector<boost::float64_t>(n));
}

bool
robot_path::find(const std::string &v) const
{
    v_it_t f, t;
    for (boost::tie(f, t) = vertices(graph_); f != t; ++f)
        if (v == name(*f))
            break;
    return !(f == t);
}

std::string
robot_path::name(const robot_path::v_desc_t &i) const
{
    return get(vertex_name, graph_, i);
}
robot_path::v_desc_t
robot_path::index(const std::string &n) const
{
    v_it_t f, t;
    for (boost::tie(f, t) = vertices(graph_); f != t; ++f)
        if (n == name(*f))
            break;
    return (f == t ? NULL_V : *f);
}

bool
robot_path::is_articulation(const v_desc_t &v)
{
    std::vector<v_desc_t> art_p;
    articulation_points(graph_, std::back_inserter(art_p));
    for (size_t i = 0; i < art_p.size(); ++i)
        if (v == art_p[i])
            return true;
    return false;
}
bool
robot_path::is_articulation(const e_desc_t &e)
{
    if (count[e] > 0)
        return false;
    /// 没有重边的时候，要判断是否在圈中。不在圈中，有去无回
    e_comp_t comp = get(edge_component, graph_);
    biconnected_components(graph_, comp);
    e_it_t f, t;
    for (boost::tie(f, t) = edges(graph_); f != t; ++f)
        if (comp[e] == comp[*f] && e != *f) // 割边单独属于一个联通分支
            return false;
    return true;
}

bool
robot_path::full_shortest_distance()
{
    assign_distance(v_num());
    return johnson_all_pairs_shortest_paths(graph_, distance_);
}

boost::float64_t
robot_path::shortest_path(const v_desc_t &u, const v_desc_t &v, std::list<v_desc_t> &r)
{
    std::vector<v_desc_t> prior(v_num());
    std::vector<boost::float64_t> d(v_num());
    dijkstra_shortest_paths(
        graph_, u,
        predecessor_map(
            boost::make_iterator_property_map(
                prior.begin(), get(vertex_index, graph_))).
        distance_map(
            boost::make_iterator_property_map(
                d.begin(), get(vertex_index, graph_))));
    for (v_desc_t i = v; i != u; i = prior[i])
        r.push_front(i);
    r.push_front(u);                    // 不需要列出起点

    return d[v];
}

std::list<robot_path::e_desc_t>
robot_path::shortest_path(const e_desc_t &u, const e_desc_t &v)
{
    std::list<e_desc_t> es;
    std::list<v_desc_t> path;

    shortest_path(source(u, graph_), source(v, graph_), path);
    v_desc_t sv = path.front();
    path.pop_front();
    while (!path.empty()) {
        v_desc_t ev = path.front();
        tri_edge_t te = LC_THIS[vv_edge_t(sv, ev)];
        if (te.second) {
            es.push_back(te.first);
        }
        sv = ev;
        path.pop_front();
    }
    shortest_path(target(u, graph_), target(v, graph_), path);
    sv = path.front();
    path.pop_front();
    while (!path.empty()) {
        v_desc_t ev = path.front();
        tri_edge_t te = LC_THIS[vv_edge_t(sv, ev)];
        if (te.second) {
            es.push_back(te.first);
        }
        sv = ev;
        path.pop_front();
    }

    es.sort();
    es.unique();
    return es;
}

robot_path
robot_path::extence(const std::vector<v_desc_t> &vs)
{
    robot_path ext;
    if (full_shortest_distance())
        for (size_t i = 0; i < vs.size() - 1; ++i)
            for (size_t j = i + 1; j < vs.size(); ++j)
                ext.ins_edge(
                    comm_edge_t(name(vs[i]), name(vs[j]),
                                distance_[vs[i]][vs[j]]));
    clear_distance();
    return ext;
}

std::list<robot_path::e_desc_t>
robot_path::min_conn_closer_ext()
{
    std::list<e_desc_t> closer;
    std::map<e_desc_t, bool> flag;
    do { // 计算本次任务包含的边集合
        e_it_t f, t;
        for (boost::tie(f, t) = edges(graph_); f != t; ++f) {
            if (get(edge_color, graph_, *f))
                closer.push_back(*f);
            flag[*f] = false;
        }
        std::cout << "任务包含的边数: " << closer.size() << " ";

        if (closer.size() < 3) {
            closer.clear();
            return closer;
        }
    } while (0);

    do {
        std::vector<e_desc_t> vec;
        std::list<e_desc_t>::iterator it;
        for (it = closer.begin(); it != closer.end(); ++it)
            vec.push_back(*it);
        closer.clear();

        for (size_t i = 0; i < vec.size(); ++i) {
            for (size_t j = i + 1; j < vec.size(); ++j) {
                std::list<e_desc_t> path = shortest_path(vec[i], vec[j]);
                for (it = path.begin(); it != path.end(); ++it)
                    flag[*it] = true;
                flag[vec[i]] = true;
            }
        }
    } while (0);

    std::map<e_desc_t, bool>::iterator mt;
    for (mt = flag.begin(); mt != flag.end(); ++mt)
        if (mt->second)
            closer.push_back(mt->first);
    return closer;
}

std::list<robot_path::e_desc_t>
robot_path::min_conn_closer()
{
    std::list<e_desc_t> closer;
    std::map<e_desc_t, bool> flag;

    do { /// 先找出任务包含的边
        e_it_t f, t;
        for (boost::tie(f, t) = edges(graph_); f != t; ++f) {
            if (get(edge_color, graph_, *f))
                closer.push_back(*f);
            flag[*f] = false;
        }
        std::cout << "任务包含的边数: " << closer.size() << std::endl;
    } while (0);

    if (closer.size() < 3) {
        closer.clear();
        return closer;
    }

    e_desc_t se = closer.back();
    flag[se] = true;

    e_desc_t tmpe = closer.back();
    size_t e_count = 0;
    std::list<e_desc_t>::iterator it, jt;
    for (it = closer.begin(); it != closer.end(); ++it) {
        e_desc_t ne = *it;
        if (!flag[ne]) {
            std::list<e_desc_t> path = shortest_path(se, ne);
            for (jt = path.begin(); jt != path.end(); ++jt)
                flag[*jt] = true;
            flag[ne] = true;

            if (path.size() > e_count) {
                e_count = path.size();
                tmpe = ne;
            }
        }
    }

    for (it = closer.begin(); it != closer.end(); ++it) {
        e_desc_t ne = *it;
        std::list<e_desc_t> path = shortest_path(tmpe, ne);
        for (jt = path.begin(); jt != path.end(); ++jt)
            flag[*it] = true;
        flag[ne] = true;
    }

    closer.clear();
    std::map<e_desc_t, bool>::iterator i;
    for (i = flag.begin(); i != flag.end(); ++i)
        if (i->second)
            closer.push_back(i->first);
    return closer;
}

void
robot_path::matching(std::list<str_edge_t> &mate)
{
    std::list<e_desc_t> es;
    std::map<v_desc_t, bool> vr;

    e_it_t f, t;
    for (boost::tie(f, t) = edges(graph_); f != t; ++f) {
        es.push_back(*f);
        vr[source(*f, graph_)] = false;
        vr[target(*f, graph_)] = false;
    }

    while (es.size() > 0) {
        std::list<e_desc_t>::iterator it, r = es.begin();
        for (it = es.begin(); it != es.end(); ++it)
            if (compare(*it, *r) < 0)
                r = it;

        v_desc_t u = source(*r, graph_), v = target(*r, graph_);
        mate.push_back(str_edge_t(name(u), name(v)));
        vr[u] = true;
        vr[v] = true;

        for (it = es.begin(); it != es.end(); ) {
            u = source(*it, graph_);
            v = target(*it, graph_);

            if (vr[u] || vr[v])
                it = es.erase(it);
            else
                ++it;
        }
    }
}

void robot_path::euler()
{
    e_it_t ef, et;
    std::map<e_desc_t, size_t> c;
    std::map<e_desc_t, size_t>::iterator it;
    for (boost::tie(ef, et) = edges(graph_); ef != et; ++ef) {
        c[*ef] = 0;
    }

    std::list<str_edge_t> mate;
    do {
        std::vector<v_desc_t> v_odds;
        v_it_t f, t;
        for (boost::tie(f, t) = vertices(graph_); f != t; ++f) {
            if (degree(*f) % 2 == 1) {
                v_odds.push_back(*f);
            }
        }
        extence(v_odds).matching(mate);
    } while(0);

    std::list<v_desc_t> path;
    while (!mate.empty()) {
        vv_edge_t e = vv_edge_t(index(mate.back().first), index(mate.back().second));
        shortest_path(e.first, e.second, path);

        v_desc_t u = path.front();
        path.pop_front();

        while (!path.empty()) {
            v_desc_t v = path.front();
            tri_edge_t tri_e = LC_THIS[vv_edge_t(u, v)];
            if (tri_e.second) {
                c[tri_e.first] += 1;
            } else {
                std::cerr << __FUNCTION__ << ": " << __LINE__
                          << "-> has no this edge: "
                          << "("
                          << name(u) << "," << name(v)
                          << ")"
                          << std::endl;
            }

            u = v;
            path.pop_front();
        }

        mate.pop_back();
    }

    e_comp_t comp = get(edge_component, graph_);
    do { /// 运算割边，为fluery选择做准备，替代距离最短方式
        std::map<size_t, size_t> bc;
        biconnected_components(graph_, comp);
        for (boost::tie(ef, et) = edges(graph_); ef != et; ++ef) {
            size_t ic = comp[*ef];
            if (bc.find(ic) == bc.end()) {
                bc[ic] = 1;
            } else {
                bc[ic] += 1;
            }
        }

        for (boost::tie(ef, et) = edges(graph_); ef != et; ++ef) {
            put(edge_art, graph_, *ef, (1 == bc[comp[*ef]]));
        }

        for (it = c.begin(); it != c.end(); ++it) {
            count[it->first] = it->second % 2;
        }
    } while (0);
}


std::list<std::string>
robot_path::fluery(const v_desc_t &s, const v_desc_t &e)
{
    std::list<std::string> order;
    if (s != e) { /// 暂不支持指定终点，后续可以扩展
        //        v_desc_t vin = ins_vertex(in), vout = ins_vertex(out);
        //        add_edge(vin, s, 1, graph_);
        //        add_edge(vout, e, 1, graph_);
    }

    v_desc_t m = s;
    while (degree(m) != 0) {
        order.push_back(name(m));

        std::list<e_desc_t> adj_e, cut;
        graph_traits<graph_t>::out_edge_iterator f, t;
        for (boost::tie(f, t) = out_edges(m, graph_); f != t; ++f)
            if (get(edge_art, graph_, *f))
                cut.push_back(*f);
            else
                adj_e.push_back(*f);

        cut.sort(boost::bind(&robot_path::less, this, _1, _2));
        adj_e.sort(boost::bind(&robot_path::less, this, _1, _2));
        adj_e.splice(adj_e.begin(), cut);

        e_desc_t e = adj_e.front();
        std::list<e_desc_t>::iterator it;
        for (it = adj_e.begin(); it != adj_e.end(); ++it) {
            if (!is_articulation(*it)) { /// 最后选择割边
                e = *it;
                break;
            }
        }

        v_desc_t u = source(e, graph_);
        v_desc_t v = target(e, graph_);
        m = (m == u) ? v : u;

        if (0 == count[e])
            remove_edge(e, graph_);
        else
            count[e] = count[e] - 1;
    }

    std::cout << e_num() << std::endl;
    order.push_back(name(m));
    return order;
}

std::vector<comm_edge_t>
robot_path::optimal_path(const std::string &s, const std::string &e,
                         std::vector<comm_edge_t> &es)
{
    std::cout << "StartPoint: " << s << "  >>";
    std::list<v_desc_t> vs;
    std::vector<comm_edge_t> order;
    std::map<e_desc_t, bool> flag;
    std::map<std::string, v_desc_t> vmap;

    if (es.size() > k_vmax || es.size() < 3) {
        return order;
    }

    do { /// 构建地图
        ins_vertex(s);
        ins_vertex(e);
        std::vector<comm_edge_t>::iterator it;
        for (it = es.begin(); it != es.end(); ++it) {
            tri_edge_t te = ins_edge(*it);
            flag[te.first] = false;
        }
    } while(0);
    // of_graph();

    do { /// 建立顶点索引与名称的双向map
        v_it_t f, t;
        for (boost::tie(f, t) = vertices(graph_); f != t; ++f) {
            vmap[name(*f)] = *f;
        }
    } while(0);

    do { /// 修改为从一条边的起点到达其他各边起点的最短路径
        robot_path mcc;
        std::list<e_desc_t> es = min_conn_closer_ext();
        std::list<e_desc_t>::iterator it;
        for (it = es.begin(); it != es.end(); ++it) {
            comm_edge_t e(name(source(*it, graph_)),
                          name(target(*it, graph_)),
                          LC_THIS[*it]);
            mcc.ins_edge(e);
        }

        es.clear();
        mcc.euler();
        //mcc.of_graph();

        v_desc_t start_v = mcc.index(s);
        std::list<std::string> vv = mcc.fluery(start_v, start_v);

        v_desc_t u = index(vv.front());
        vv.pop_front();
        while (!vv.empty()) {
            v_desc_t v = index(vv.front());
            tri_edge_t te = LC_THIS[vv_edge_t(u, v)];
            if (te.second) {
                es.push_back(te.first);
            } else {
                std::cout << "no this edge with v 1: " << name(u) << ", " << name(v) << std::endl;
            }

            u = v;
            vv.pop_front();
        }

#if 1
        for (it = es.begin(); it != es.end(); ) {
            if (get(edge_color, graph_, *it)) { /// 只保留任务中指定的边
                if (flag[*it]) { /// 去重复
                    it = es.erase(it);
                } else {
                    order.push_back(
                        comm_edge_t(name(source(*it, graph_)),
                                    name(target(*it, graph_)),
                                    LC_THIS[*it]));
                    flag[*it] = true;
                    ++it;
                }
            } else {
                it = es.erase(it);
            }
        }
#else
        for (it = es.begin(); it != es.end(); ++it) {
            order.push_back(
                comm_edge_t(name(source(*it, graph_)),
                            name(target(*it, graph_)), LC_THIS[*it]));
        }
#endif
    } while (0);

    //of_graph();
    return order;
}

robot_path::v_desc_t
robot_path::ins_vertex(const std::string &v)
{
    v_desc_t vdesc = index(v);
    if (NULL_V == vdesc)
        vdesc = add_vertex(v, graph_);
    return vdesc;
}
robot_path::tri_edge_t
robot_path::ins_edge(const comm_edge_t &e)
{
    v_desc_t u = ins_vertex(e.source_);
    v_desc_t v = ins_vertex(e.target_);
    tri_edge_t te = add_edge(u, v, e.d_, graph_);
    if (te.second) {
        put(edge_color, graph_, te.first, e.task_);
    }

    return te;
}

void robot_path::of_graph()
{
    std::vector<v_desc_t> art_points;
    articulation_points(graph_, std::back_inserter(art_points));

    std::ofstream dot_file("./path.org");
    dot_file << "#+begin_src dot :file instruction_schedule.png :cmdline -Kdot -Tpng\n";
    dot_file << "digraph D {\n"
             << "rankdir=TB;\n"
             << "layout=fdp;\n"
             << "edge[fontname=\"Lucida Math Std\"];\n"
             << "node[shape=\"circle\", color=blue, style=filled];\n";
    for (std::vector<v_desc_t>::iterator it = art_points.begin();
         it != art_points.end(); ++it)
        dot_file << name(*it) << "[color=orchid];\n";

    e_it_t ef, et;
    for (boost::tie(ef, et) = edges(graph_); ef != et; ++ef) {
        e_desc_t e = *ef;
        v_desc_t u = source(e, graph_), v = target(e, graph_);

        dot_file << name(u) << " -> " << name(v)
                 << "[label=\"" << LC_THIS[e] << "\""
                 << ", dir=none";
        if (get(edge_art, graph_, e))
            dot_file << ", color=\"red\"";
        else
            dot_file << ", color=\"black\"";
        if (get(edge_color, graph_, e))
            dot_file << ", style=bold";
        dot_file << "];\n";
    }

    dot_file << "}\n";
    dot_file << "#+end_src";
}

robot_path::v_desc_t
robot_path::nearest(const v_desc_t &u, const std::vector<v_desc_t> &v)
{
    std::vector<v_desc_t> prior(v_num());
    std::vector<boost::float64_t> distance;
    distance.assign(v_num(), std::numeric_limits<boost::float64_t>::max());

    dijkstra_shortest_paths(
        graph_, u,
        predecessor_map(
            boost::make_iterator_property_map(
                prior.begin(), get(vertex_index, graph_))).distance_map(
                    boost::make_iterator_property_map(
                        distance.begin(), get(vertex_index, graph_))));

    v_desc_t n = u;
    boost::float64_t d = std::numeric_limits<boost::float64_t>::max();
    for (size_t i = 0; i < v.size(); ++i) {
        if (v[i] != u && distance[v[i]] < d) {
            n = v[i];
            d = distance[v[i]];
        }
    }

    return n;
}

#if (defined(_WIN32) || defined(_WIN64))
#include <boost/lexical_cast.hpp>

size_t __stdcall calc_optimal_path(unsigned int s, size_t size, out_edge_t *edges)
{
    robot_path graph;
    std::vector<comm_edge_t> in_edges;

    if (size <= 0)
        return 0;

    size_t i = 0;
    for (i = 0; i < size; ++i) {
        comm_edge_t e(
            boost::lexical_cast<std::string>(edges[i].in_),
            boost::lexical_cast<std::string>(edges[i].out_),
            edges[i].d_);
        e.task(1 == edges[i].t_);
        in_edges.push_back(e);
    }

    std::string u = boost::lexical_cast<std::string>(s);
    in_edges = graph.optimal_path(u, u, in_edges);

    for (i = 0; i < in_edges.size(); ++i) {
        edges[i].in_ = boost::lexical_cast<unsigned int>(in_edges[i].source_);
        edges[i].out_ = boost::lexical_cast<unsigned int>(in_edges[i].target_);
        edges[i].d_ = in_edges[i].d_;
    }

    return in_edges.size();
}
#endif /* WIN */