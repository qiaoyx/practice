#!/bin/sh
scan-build gcc -o tsp tsp.cpp map.cpp  main.cpp -lpthread -lIce -lIceUtil -lboost_graph -L/opt/Ice-3.6.2/lib/x86_64-linux-gnu -I/opt/Ice-3.6.2/include -lstdc++ -rdynamic
