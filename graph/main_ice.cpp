#include <iomanip>
#include <iostream>
#include <fstream>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <Ice/Ice.h>
#include <Ice/ServantLocator.h>
#include "tsp.hpp"
#include "map.h"

#include <boost/date_time/posix_time/posix_time.hpp>

using namespace RobotPath;
#define print std::cout
#define newl  std::endl

class OptimalPathI : public OptimalPath
{
public:
    RobotPath::edgeArray calcOptimalPath (
        const std::string &start,
        const RobotPath::edgeArray &es,
        const Ice::Current& ie /*=  Ice::noExplicitCurrent */) {
        robot_path graph;
        std::vector<comm_edge_t> edges;
        edgeArray::const_iterator it;
        for (it = es.begin(); it != es.end(); ++it) {
            // print << it->source << ", " << it->target << ": " << it->d << newl;
            comm_edge_t e(it->source, it->target, it->d);
            e.task(it->task);
            edges.push_back(e);
        }

        posix_time::ptime time = posix_time::second_clock::local_time();
        edges = graph.optimal_path(start, start, edges);
        print << "Time: " << posix_time::second_clock::local_time() - time << " s" << newl;

        edgeArray res;
        for (size_t i = 0; i < edges.size(); ++i) {
            edgeNode e;
            e.source = edges[i].source_;
            e.target = edges[i].target_;
            e.d = edges[i].d_;
            e.task = edges[i].task_;
            res.push_back(e);
        }

        return res;
    }
};

class PathServantLocator : public Ice::ServantLocator
{
public:
    PathServantLocator() {}
    virtual ~PathServantLocator() {}

    virtual Ice::ObjectPtr
    locate(const Ice::Current& c, Ice::LocalObjectPtr& cookie) {
        IceUtil::Mutex::Lock lock(mutex_);
        Ice::ObjectPtr servant = c.adapter->find(c.id);
        if (!servant) {
            servant = new OptimalPathI;
            c.adapter->add(servant, c.id);
        }
        return servant;
    }
    virtual void
    finished(const Ice::Current& c, const Ice::ObjectPtr& servant,
             const Ice::LocalObjectPtr& cookie) {
        return;
    }
    virtual void
    deactivate(const std::string& category) {
        return;
    }

private:
    IceUtil::Mutex mutex_;
};

class MyIceApp : public Ice::Application
{
public:
    virtual int run(int argc, char* argv[]) {
        Ice::CommunicatorPtr ic = communicator();
        Ice::ObjectAdapterPtr adapter
            //= ic->createObjectAdapterWithEndpoints("RobotPathAdapter", "default -h 0.0.0.0 -p 9090");
            = ic->createObjectAdapter("RobotPathAdapter");
        Ice::ObjectPtr object = new OptimalPathI;
        adapter->add(object, ic->stringToIdentity("RobotPath"));
        // adapter->addServantLocator(
        //     Ice::ServantLocatorPtr(new PathServantLocator), "RobotPath");
        //adapter->addDefaultServant(new OptimalPathI, "RobotPath");
        adapter->activate();
        ic->waitForShutdown();
        return 0;
    }
};

int main(int argc, char *argv[])
{
    MyIceApp app;
    return app.main(argc, argv, "config.server");
}
